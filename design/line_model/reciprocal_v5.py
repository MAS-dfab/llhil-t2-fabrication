# -*- coding: utf-8 -*-
"""
Reciprocal Frame Solver

Usage:
    from design.line_model.reciprocal_v5 import reciprocal_from_subgraph
    result = reciprocal_from_subgraph(subgraph, engage_len=0.3)
"""

import math
from compas.geometry import (
    Point, Vector, Line,
    cross_vectors, dot_vectors, length_vector, normalize_vector,
    add_vectors, subtract_vectors, scale_vector,
    distance_point_point, intersection_line_line,
)
from compas.datastructures import Graph
import numpy as np
from scipy.optimize import least_squares


# ==============================================================================
# ReciprocalSolver
# ==============================================================================

def _intersect_lines_3d(p1, d1, p2, d2, normal):
    """Find intersection of two lines projected onto their common plane."""
    e1 = normalize_vector(d1)
    e2 = cross_vectors(normal, e1)
    if length_vector(e2) < 1e-9:
        return None
    e2 = normalize_vector(e2)
    
    det = dot_vectors(d1, e1) * (-dot_vectors(d2, e2)) - (-dot_vectors(d2, e1)) * dot_vectors(d1, e2)
    if abs(det) < 1e-9:
        return None
    
    delta = subtract_vectors(p2, p1)
    t = (dot_vectors(delta, e1) * (-dot_vectors(d2, e2)) - dot_vectors(delta, e2) * (-dot_vectors(d2, e1))) / det
    return add_vectors(p1, scale_vector(d1, t))


class ReciprocalSolver:
    """Solver for reciprocal frame using perpendicular shifts."""
    
    def __init__(self, lines, engage_len=1.0, tol=0.1):
        self.engage_len = engage_len
        self.tol = tol
        self.beams = []
        self.vertices = []
        self.fixed_ends = []
        
        # Extract beams from lines
        for i, line in enumerate(lines):
            p0 = [line.start.x, line.start.y, line.start.z]
            p1 = [line.end.x, line.end.y, line.end.z]
            if distance_point_point(p0, p1) < 1e-9:
                continue
            self.beams.append({'key': i, 'pts': [p0, p1], 'original': [p0, p1], 'shift': 0.0})
        
        self.fixed_ends = [{0: True, 1: True} for _ in self.beams]
        self._find_vertices()

    def calculate_ideal_targets(self, rotation_sign):
        """
        Calculates perfect zero-gap intersections based on CURRENT angles.
        Returns a dictionary mapping (beam_index, end_index) to new coordinates.
        """
        intended_moves = {}
        
        for vi, V in enumerate(self.vertices):
            m = len(V['fan'])
            shifts_v = []
            
            for k in range(m):
                bi, ei = V['fan'][k]
                shift_dist = self.beams[bi]['shift']
                shifts_v.append(shift_dist * rotation_sign)
            
            pts = self._vertex_nexus(vi, shifts_v)
            
            for k in range(m):
                bi, ei = V['fan'][k]
                
                # FIX: Assign the correct intersection based on the pinwheel direction
                target_pt = pts[k] if rotation_sign > 0 else pts[(k - 1) % m]
                
                if target_pt:
                    intended_moves[(bi, ei)] = target_pt
                    
        return intended_moves
    
    def _find_vertices(self):
        """Find multi-way vertices by clustering beam endpoints."""
        endpoints = []
        for i, b in enumerate(self.beams):
            endpoints.append((b['original'][0], i, 0))
            endpoints.append((b['original'][1], i, 1))
        
        clusters = []
        for pt, bi, ei in endpoints:
            found = False
            for c in clusters:
                if distance_point_point(c['point'], pt) < self.tol:
                    c['lines'].append((bi, ei))
                    found = True
                    break
            if not found:
                clusters.append({'point': pt, 'lines': [(bi, ei)]})
        
        for c in clusters:
            if len(c['lines']) < 3:
                continue
            
            fan, normal = self._cyclic_sort(c['point'], c['lines'])
            sides = []
            for (bi, ei) in fan:
                d = self._outgoing_dir(bi, ei)
                s = cross_vectors(normal, d)
                s = normalize_vector(s) if length_vector(s) > 1e-9 else [0, 1, 0]
                sides.append(s)
            
            for (bi, ei) in fan:
                self.fixed_ends[bi][ei] = False
            
            self.vertices.append({'point': c['point'], 'fan': fan, 'normal': normal, 'side': sides})
    
    def _outgoing_dir(self, bi, ei):
        b = self.beams[bi]
        d = subtract_vectors(b['original'][1 - ei], b['original'][ei])
        return normalize_vector(d)
    
    def _cyclic_sort(self, center, fan):
        dirs = [self._outgoing_dir(bi, ei) for (bi, ei) in fan]
        
        normal = [0, 0, 0]
        for i in range(len(dirs)):
            for j in range(i + 1, len(dirs)):
                c = cross_vectors(dirs[i], dirs[j])
                if length_vector(c) > 1e-3:
                    c = normalize_vector(c)
                    if length_vector(normal) < 1e-3:
                        normal = c
                    elif dot_vectors(normal, c) < 0:
                        normal = subtract_vectors(normal, c)
                    else:
                        normal = add_vectors(normal, c)
        
        normal = normalize_vector(normal) if length_vector(normal) > 1e-3 else [0, 0, 1]
        
        ref = dirs[0]
        proj = dot_vectors(ref, normal)
        ref = subtract_vectors(ref, scale_vector(normal, proj))
        ref = normalize_vector(ref) if length_vector(ref) > 1e-6 else [1, 0, 0]
        
        def angle(idx):
            d = dirs[idx]
            dp = subtract_vectors(d, scale_vector(normal, dot_vectors(d, normal)))
            if length_vector(dp) < 1e-6:
                return 0.0
            dp = normalize_vector(dp)
            a = math.acos(max(-1.0, min(1.0, dot_vectors(dp, ref))))
            if dot_vectors(cross_vectors(ref, dp), normal) < 0:
                a = 2 * math.pi - a
            return a
        
        order = sorted(range(len(fan)), key=angle)
        return [fan[i] for i in order], normal
    
    def solve(self, rotation_sign=-1):
        var_index = {}
        for vi, V in enumerate(self.vertices):
            for k in range(len(V['fan'])):
                var_index[(vi, k)] = len(var_index)
        
        if not var_index:
            return {'converged': True, 'cost': 0.0, 'iterations': 0}
        
        n_vars = len(var_index)
        x0 = np.zeros(n_vars)
        for vi, V in enumerate(self.vertices):
            m = len(V['fan'])
            s_init = self.engage_len / (2.0 * math.sin(math.pi / m)) * rotation_sign
            for k in range(m):
                x0[var_index[(vi, k)]] = s_init
        
        bound = 2.0 * self.engage_len
        
        # Determine strict bounds based on rotation sign
        if rotation_sign > 0:
            lower_bound = np.zeros(n_vars) + 1e-9
            upper_bound = np.full(n_vars, bound)
        else:
            lower_bound = np.full(n_vars, -bound)
            upper_bound = np.zeros(n_vars) - 1e-9

        # Ensure initial guess is within the strict bounds
        x0 = np.clip(x0, lower_bound, upper_bound)
        
        result = least_squares(
            lambda x: self._residuals(x, var_index, rotation_sign),
            x0, bounds=(lower_bound, upper_bound), method='trf', ftol=1e-10, xtol=1e-10, max_nfev=200
        )
        
        self._apply_solution(result.x, var_index)
        return {'converged': result.success, 'cost': float(result.cost), 'iterations': int(result.nfev)}
    
    def _residuals(self, x, var_index, rotation_sign):
        res = []
        for vi, V in enumerate(self.vertices):
            m = len(V['fan'])
            sv = [x[var_index[(vi, k)]] for k in range(m)]
            pts = self._vertex_nexus(vi, sv)
            
            for k in range(m):
                P_prev, P_curr = pts[(k - 1) % m], pts[k]
                if P_prev is None or P_curr is None:
                    res.append(1e3 * self.engage_len)
                else:
                    res.append(distance_point_point(P_prev, P_curr) - self.engage_len)
        
        for v in x:
            res.append(0.1 * abs(v) if v * rotation_sign < 0 else 0.0)
        return np.array(res)
    
    def _vertex_nexus(self, vi, shifts_v):
        V = self.vertices[vi]
        m, n = len(V['fan']), V['normal']
        pts = []
        for k in range(m):
            o1, d1 = self._shifted_origin(vi, k, shifts_v[k])
            o2, d2 = self._shifted_origin(vi, (k + 1) % m, shifts_v[(k + 1) % m])
            pts.append(_intersect_lines_3d(o1, d1, o2, d2, n))
        return pts
    
    def _shifted_origin(self, vi, k, s):
        V = self.vertices[vi]
        bi, ei = V['fan'][k]
        return add_vectors(V['point'], scale_vector(V['side'][k], s)), self._outgoing_dir(bi, ei)
    
    def _apply_solution(self, shifts, var_index):
        for vi, V in enumerate(self.vertices):
            m = len(V['fan'])
            pts = self._vertex_nexus(vi, [shifts[var_index[(vi, k)]] for k in range(m)])
            for k in range(m):
                bi, ei = V['fan'][k]
                if pts[k]:
                    self.beams[bi]['pts'][ei] = pts[k]
    
    def trim_to_neighbors(self, max_search=None):
        """
        Postprocess: snap each free endpoint to the closest intersection with
        another beam, measured along the beam's direction from its OTHER end.
        """
        if max_search is None:
            max_search = 5.0 * self.engage_len
        
        n = len(self.beams)
        coplanar_tol = max(0.1 * self.engage_len, 1e-3)
        
        for bi in range(n):
            for ei in (0, 1):
                if self.fixed_ends[bi][ei]:
                    continue
                
                anchor = self.beams[bi]['pts'][1 - ei]
                current_end = self.beams[bi]['pts'][ei]
                d = subtract_vectors(current_end, anchor)
                cur_t = length_vector(d)
                if cur_t < 1e-9:
                    continue
                d = normalize_vector(d)
                
                # Create extended line for intersection testing
                far = add_vectors(anchor, scale_vector(d, cur_t + max_search))
                line_i = Line(Point(*anchor), Point(*far))
                
                min_t = 0.1 * cur_t
                best_t = None
                best_delta = None
                
                for bj in range(n):
                    if bj == bi:
                        continue
                    q1, q2 = self.beams[bj]['pts']
                    if distance_point_point(q1, q2) < 1e-9:
                        continue
                    line_j = Line(Point(*q1), Point(*q2))
                    
                    result = intersection_line_line(line_i, line_j)
                    if result is None:
                        continue
                    
                    p_on_i, p_on_j = result
                    if p_on_i is None or p_on_j is None:
                        continue
                    
                    miss = distance_point_point(p_on_i, p_on_j)
                    if miss > coplanar_tol:
                        continue
                    
                    # Compute parameter t along line_i
                    t_dist = distance_point_point(anchor, p_on_i)
                    if t_dist < min_t:
                        continue
                    
                    # Check if p_on_j is within line_j bounds
                    line_j_len = distance_point_point(q1, q2)
                    tj = distance_point_point(q1, p_on_j) / line_j_len if line_j_len > 1e-9 else 0
                    
                    # Check direction
                    test_d = subtract_vectors(p_on_j, q1)
                    line_d = subtract_vectors(q2, q1)
                    if dot_vectors(test_d, line_d) < 0:
                        tj = -tj
                    
                    if tj < -0.001 or tj > 1.001:
                        continue
                    
                    # Pick closest to current endpoint
                    delta = abs(t_dist - cur_t)
                    if best_delta is None or delta < best_delta:
                        best_delta = delta
                        best_t = t_dist
                
                if best_t is not None and abs(best_t - cur_t) > 1e-6:
                    new_end = add_vectors(anchor, scale_vector(d, best_t))
                    self.beams[bi]['pts'][ei] = new_end
    
    def to_lines(self):
        return [Line(Point(*b['pts'][0]), Point(*b['pts'][1])) for b in self.beams]
    
    def get_nexus_lines(self):
        lines = []
        for V in self.vertices:
            pts = [Point(*self.beams[bi]['pts'][ei]) for (bi, ei) in V['fan']]
            m = len(pts)
            for k in range(m):
                lines.append(Line(pts[k], pts[(k + 1) % m]))
        return lines


# ==============================================================================
# Helpers
# ==============================================================================

def _get_pt(subgraph, node):
    """Get point from node."""
    pt = subgraph.node_attribute(node, 'point')
    if pt is None:
        return Point(subgraph.node_attribute(node, 'x'),
                    subgraph.node_attribute(node, 'y'),
                    subgraph.node_attribute(node, 'z') or 0.0)
    return Point(pt.x, pt.y, pt.z) if hasattr(pt, 'x') else Point(*pt)


def _get_line(subgraph, edge):
    """Get line from edge."""
    u, v = edge
    shifted = subgraph.edge_attribute((u, v), 'shifted_lines')
    if shifted and isinstance(shifted, Line):
        return shifted
    if shifted:
        return Line(Point(*shifted[0]), Point(*shifted[1]))

    line = subgraph.edge_attribute((u, v), 'line')
    if line and isinstance(line, Line):
        return line
    if line:
        return Line(Point(*line[0]), Point(*line[1]))
    return Line(_get_pt(subgraph, u), _get_pt(subgraph, v))


def _as_set(value):
    if value is None:
        return None
    if isinstance(value, str):
        return {value}
    return set(value)


def _is_primary_hierarchy(hierarchy):
    return (
        hierarchy == 'primary'
        or hierarchy == 'primary_orthogonal'
        or (isinstance(hierarchy, str) and hierarchy.startswith('primary_diagonal_'))
    )


def _hierarchy_matches(hierarchy, active_hierarchies=None):
    active_hierarchies = _as_set(active_hierarchies)
    if active_hierarchies is None:
        return _is_primary_hierarchy(hierarchy)

    for wanted in active_hierarchies:
        if hierarchy == wanted:
            return True
        if wanted == 'primary' and _is_primary_hierarchy(hierarchy):
            return True
        if wanted == 'primary_diagonal' and isinstance(hierarchy, str) and hierarchy.startswith('primary_diagonal_'):
            return True
    return False


def _is_reciprocal_edge(subgraph, edge, active_hierarchies=None, active_categories=None):
    hierarchy = subgraph.edge_attribute(edge, 'hierarchy')
    if hierarchy == 'double':
        return True

    active_categories = _as_set(active_categories)
    hierarchy_ok = _hierarchy_matches(hierarchy, active_hierarchies)
    category_ok = active_categories is None or subgraph.edge_attribute(edge, 'e_category') in active_categories

    if active_hierarchies is not None and active_categories is not None:
        return hierarchy_ok or (category_ok and (hierarchy is None or _is_primary_hierarchy(hierarchy)))

    if not hierarchy_ok:
        return False
    if active_categories is None:
        return True
    return category_ok


def _is_diagonal_pass(active_hierarchies=None, active_categories=None):
    hierarchies = _as_set(active_hierarchies)
    if hierarchies:
        for hierarchy in hierarchies:
            if hierarchy == 'primary_diagonal':
                return True
            if isinstance(hierarchy, str) and hierarchy.startswith('primary_diagonal_'):
                return True

    categories = _as_set(active_categories)
    if categories:
        diagonal_categories = {'default_diagonal', 'moved_diagonal'}
        if categories and categories.issubset(diagonal_categories):
            return True

    return False


def _is_orthogonal_stop_edge(subgraph, edge):
    hierarchy = subgraph.edge_attribute(edge, 'hierarchy')
    category = subgraph.edge_attribute(edge, 'e_category')
    return hierarchy == 'primary_orthogonal' or (hierarchy == 'primary' and category == 'orthogonal')


def _is_plane_seed_edge(subgraph, edge):
    hierarchy = subgraph.edge_attribute(edge, 'hierarchy')
    return hierarchy == 'double' or _is_primary_hierarchy(hierarchy)


def _node_touches_orthogonal_stop(subgraph, node):
    for edge in subgraph.edges():
        if node in edge and _is_orthogonal_stop_edge(subgraph, edge):
            return True
    return False

def _store_shifted_lines(subgraph, edges, shifted_lines):
    for edge, line in zip(edges, shifted_lines):
        subgraph.edge_attribute(edge, 'line', line)
        subgraph.edge_attribute(edge, 'shifted_lines', line)


# ==============================================================================
# Coplanar Group Detection
# ==============================================================================

def find_coplanar_groups(subgraph, min_degree=2, plane_tol=0.1, debug=True,
                         active_hierarchies=None, active_categories=None):
    """
    Find groups of primary+double edges that lie on the same plane.
    Returns groups where 2+ edges meet at a joint.
    """
    from compas.geometry import Frame
    stop_diagonal_at_orthogonal = _is_diagonal_pass(active_hierarchies, active_categories)
    
    # Get active primary/double edges.
    edges = [(u, v) for u, v in subgraph.edges()
             if _is_reciprocal_edge(subgraph, (u, v), active_hierarchies, active_categories)]

    def edge_points(edge):
        line = _get_line(subgraph, edge)
        return line.start, line.end

    def edge_length(edge):
        p1, p2 = edge_points(edge)
        return distance_point_point([p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z])

    edges = [edge for edge in edges if edge_length(edge) > 1e-9]
    
    if debug:
        print(f"Total primary+double edges: {len(edges)}")
    
    if not edges:
        return []
    
    def edge_midpoint(edge):
        p1, p2 = edge_points(edge)
        return Point((p1.x + p2.x)/2, (p1.y + p2.y)/2, (p1.z + p2.z)/2)
    
    def edge_direction(edge):
        p1, p2 = edge_points(edge)
        d = Vector(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
        if d.length < 1e-9:
            return None
        d.unitize()
        return d
    
    def make_plane_frame(edge):
        """Create a plane frame from edge: origin=midpoint, normal=edge×Z"""
        mid = edge_midpoint(edge)
        d = edge_direction(edge)
        if d is None:
            return None, None
        
        # Normal = edge × Z
        normal = d.cross(Vector(0, 0, 1))
        if normal.length < 1e-9:
            normal = d.cross(Vector(1, 0, 0))
        normal.unitize()
        
        # Frame: X=edge dir, Y=normal×edge, Z(local)=normal
        xaxis = d
        yaxis = normal.cross(d)
        yaxis.unitize()
        
        return Frame(mid, xaxis, yaxis), normal
    
    def edge_on_plane(edge, frame, tol):
        """Check if both endpoints of edge lie on the plane"""
        p1, p2 = edge_points(edge)
        local1 = frame.to_local_coordinates(p1)
        local2 = frame.to_local_coordinates(p2)
        return abs(local1[2]) < tol and abs(local2[2]) < tol

    def edge_direction_from_node(edge, node):
        line = _get_line(subgraph, edge)
        if edge[0] == node:
            p1, p2 = line.start, line.end
        else:
            p1, p2 = line.end, line.start
        d = Vector(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
        if d.length < 1e-9:
            return None
        d.unitize()
        return d

    def node_level(node):
        level = subgraph.node_attribute(node, 'level')
        return int(level) if level is not None else 0

    def is_support_node(node):
        return (
            subgraph.node_attribute(node, 'reached') is True
            or subgraph.node_attribute(node, 'is_support') is True
            or subgraph.node_attribute(node, 'ntype') == 'support'
        )

    def edge_other(edge, node):
        if edge[0] == node:
            return edge[1]
        if edge[1] == node:
            return edge[0]
        return None

    def resolve_edge(edge):
        if subgraph.has_edge(edge):
            return edge
        reversed_edge = (edge[1], edge[0])
        if subgraph.has_edge(reversed_edge):
            return reversed_edge
        return None

    def node_sort_key(node):
        support_id = subgraph.node_attribute(node, 'support_id')
        if support_id is None:
            support_id = 10**9
        return (node_level(node), int(support_id), int(node))

    def make_plane_frame_from_node(edge, source_node):
        """Create a directed plane from an edge, oriented away from source_node."""
        line = _get_line(subgraph, edge)
        if edge[0] == source_node:
            p0, p1 = line.start, line.end
        elif edge[1] == source_node:
            p0, p1 = line.end, line.start
        else:
            return make_plane_frame(edge)

        d = Vector(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z)
        if d.length < 1e-9:
            return None, None
        d.unitize()

        normal = d.cross(Vector(0, 0, 1))
        if normal.length < 1e-9:
            normal = d.cross(Vector(1, 0, 0))
        normal.unitize()

        yaxis = normal.cross(d)
        yaxis.unitize()
        return Frame(p0, d, yaxis), normal

    def edge_in_directed_plane(edge, frame, tol):
        if not edge_on_plane(edge, frame, tol):
            return False
        midpoint = edge_midpoint(edge)
        local_mid = frame.to_local_coordinates(midpoint)
        return local_mid[0] >= -tol

    def collect_support_nodes():
        return sorted([node for node in subgraph.nodes() if is_support_node(node)], key=node_sort_key)

    def collect_incident_active_edges(node, remaining_edges):
        found = []
        remaining_set = set(remaining_edges)
        for edge in subgraph.node_edges(node):
            oriented = resolve_edge(edge)
            if oriented is None:
                continue
            active_edge = oriented if oriented in remaining_set else (oriented[1], oriented[0])
            if active_edge in remaining_set:
                found.append(active_edge)
        return found

    def collect_seed_edges_from_source(node, remaining_edges):
        """Use active edges first; if none exist, use primary/double edges as direction probes."""
        active_seeds = collect_incident_active_edges(node, remaining_edges)
        if active_seeds:
            return active_seeds

        seed_edges = []
        for edge in subgraph.node_edges(node):
            oriented = resolve_edge(edge)
            if oriented is not None and _is_plane_seed_edge(subgraph, oriented):
                seed_edges.append(oriented)
        return seed_edges

    def build_directed_plane_groups(active_edges):
        """Seed planes from supports, then from nodes reached along orthogonal edges."""
        remaining = list(active_edges)
        groups_from_sources = []

        frontier = collect_support_nodes()
        visited_sources = set()

        while frontier and remaining:
            next_frontier = []
            for source_node in frontier:
                if source_node in visited_sources:
                    continue
                visited_sources.add(source_node)

                seed_edges = collect_seed_edges_from_source(source_node, remaining)
                for seed in seed_edges:
                    frame, normal = make_plane_frame_from_node(seed, source_node)
                    if frame is None:
                        continue

                    on_plane = []
                    still_remaining = []
                    for edge in remaining:
                        if edge_in_directed_plane(edge, frame, plane_tol):
                            on_plane.append(edge)
                        else:
                            still_remaining.append(edge)

                    if on_plane:
                        remaining = still_remaining
                        groups_from_sources.append((frame, normal, on_plane))
                        if debug:
                            print(f"  Directed plane from node {source_node}: {len(on_plane)} edges")

                for edge in subgraph.node_edges(source_node):
                    oriented = resolve_edge(edge)
                    if oriented is None or not _is_orthogonal_stop_edge(subgraph, oriented):
                        continue
                    other = edge_other(oriented, source_node)
                    if other is not None and other not in visited_sources:
                        next_frontier.append(other)

            frontier = sorted(set(next_frontier), key=node_sort_key)

        return groups_from_sources, remaining
    
    # Group edges by plane
    plane_groups = []
    remaining = list(edges)

    if active_hierarchies is not None or active_categories is not None:
        directed_groups, remaining = build_directed_plane_groups(edges)
        plane_groups.extend(directed_groups)
    
    while remaining:
        # Pick first edge to define a plane
        seed = remaining.pop(0)
        frame, normal = make_plane_frame(seed)
        if frame is None:
            continue
        
        # Find all edges that lie on this plane
        on_plane = [seed]
        still_remaining = []
        
        for edge in remaining:
            if edge_on_plane(edge, frame, plane_tol):
                on_plane.append(edge)
            else:
                still_remaining.append(edge)
        
        remaining = still_remaining
        plane_groups.append((frame, normal, on_plane))
    
    if debug:
        print(f"Found {len(plane_groups)} plane groups")
        if active_hierarchies is not None or active_categories is not None:
            print(f"  Directed plane groups: {len(plane_groups)}")
        for i, (f, n, edges) in enumerate(plane_groups):
            print(f"  Plane {i}: {len(edges)} edges, normal=({n.x:.2f}, {n.y:.2f}, {n.z:.2f})")
    
    # For each plane group, find joints where 3+ edges meet
    groups = []
    
    for plane_idx, (frame, normal, plane_edges) in enumerate(plane_groups):
        # Build node -> edges map for this plane
        node_edges = {}
        for edge in plane_edges:
            u, v = edge
            node_edges.setdefault(u, []).append(edge)
            node_edges.setdefault(v, []).append(edge)
        
        # Find joints with 3+ edges
        for node, node_edge_list in node_edges.items():
            if len(node_edge_list) < min_degree:
                continue
            
            joint_pt = _get_pt(subgraph, node)
            
            # Get directions from joint
            valid_node_edges = []
            dirs = []
            for e in node_edge_list:
                d = edge_direction_from_node(e, node)
                if d is None:
                    continue
                valid_node_edges.append(e)
                dirs.append(d)

            if len(valid_node_edges) < min_degree:
                continue
            
            # Cyclic sort
            sorted_edges, sorted_dirs = _cyclic_sort(valid_node_edges, dirs, normal)
            angles = _compute_angles(sorted_dirs, normal)
            
            if debug:
                print(f"\n  Plane {plane_idx}, Joint {node}: {len(valid_node_edges)} edges")
                print(f"    Angles: {[f'{a:.0f}' for a in angles]} = {sum(angles):.0f}°")
            
            # Skip if any angle > 175° (nearly collinear edges)
            # if max(angles) > 175:
            #     if debug:
            #         print(f"    SKIPPED: angle {max(angles):.0f}° > 175°")
            #     continue
            
            lines = [_get_line(subgraph, e) for e in sorted_edges]
            groups.append({
                'node': node,
                'point': joint_pt,
                'edges': sorted_edges,
                'lines': lines,
                'normal': [normal.x, normal.y, normal.z],
                'degree': len(valid_node_edges),
                'angles': angles,
            })
    
    if debug:
        print(f"\nTotal: {len(groups)} coplanar groups (joints with 2+ edges)")
    
    return groups


def _cyclic_sort(edges, dirs, normal):
    """Sort edges cyclically around normal"""
    if len(edges) < 2:
        return edges, dirs
    
    ref = dirs[0]
    p = ref.dot(normal)
    ref = Vector(ref.x - normal.x*p, ref.y - normal.y*p, ref.z - normal.z*p)
    if ref.length > 1e-6:
        ref.unitize()
    else:
        ref = Vector(1, 0, 0)
    
    def angle(i):
        d = dirs[i]
        p = d.dot(normal)
        dp = Vector(d.x - normal.x*p, d.y - normal.y*p, d.z - normal.z*p)
        if dp.length < 1e-6:
            return 0.0
        dp.unitize()
        a = math.acos(max(-1, min(1, dp.dot(ref))))
        if ref.cross(dp).dot(normal) < 0:
            a = 2 * math.pi - a
        return a
    
    order = sorted(range(len(edges)), key=angle)
    return [edges[i] for i in order], [dirs[i] for i in order]


def _compute_angles(dirs, normal):
    """Compute angles between consecutive edges"""
    angles = []
    n = len(dirs)
    for i in range(n):
        d1, d2 = dirs[i], dirs[(i+1) % n]
        p1, p2 = d1.dot(normal), d2.dot(normal)
        d1p = Vector(d1.x - normal.x*p1, d1.y - normal.y*p1, d1.z - normal.z*p1)
        d2p = Vector(d2.x - normal.x*p2, d2.y - normal.y*p2, d2.z - normal.z*p2)
        if d1p.length < 1e-6 or d2p.length < 1e-6:
            angles.append(0)
            continue
        d1p.unitize()
        d2p.unitize()
        a = math.acos(max(-1, min(1, d1p.dot(d2p))))
        if d1p.cross(d2p).dot(normal) < 0:
            a = 2 * math.pi - a
        angles.append(math.degrees(a))
    return angles


def _line_end_at_node(edge, node):
    """Return endpoint index (0/1) of edge that coincides with the nexus node."""
    return 0 if edge[0] == node else 1


def _offset_line_at_end(line, end_index, offset):
    """Move one endpoint of a line by `offset` along its own direction."""
    p0 = [line.start.x, line.start.y, line.start.z]
    p1 = [line.end.x, line.end.y, line.end.z]

    if end_index == 0:
        d = subtract_vectors(p1, p0)
        if length_vector(d) < 1e-9:
            return line
        p0 = add_vectors(p0, scale_vector(normalize_vector(d), offset))
        return Line(Point(*p0), Point(*p1))

    d = subtract_vectors(p0, p1)
    if length_vector(d) < 1e-9:
        return line
    p1 = add_vectors(p1, scale_vector(normalize_vector(d), offset))
    return Line(Point(*p0), Point(*p1))


# ==============================================================================
# Main API
# ==============================================================================

def reciprocal_from_subgraph(subgraph, engage_len=1.0, tol=0.1, rotation_sign=+1,
                              min_degree=2, debug=True, straight_angle_threshold_deg=170.0,
                              active_hierarchies=None, active_categories=None,
                              write_attributes=True):
    """
    Reciprocalize active primary+double lines from a classified subgraph.
    
    Each coplanar group is processed SEPARATELY so vertices shared between
    groups stay FIXED unless they have 3+ edges within the SAME group.
    
    Returns dict with: shifted_lines, secondary_lines, nexus_lines, coplanar_groups, info
    """
    # Collect edges by type
    reciprocal_edges = []
    secondary_lines = []
    
    for u, v in subgraph.edges():
        if _is_reciprocal_edge(subgraph, (u, v), active_hierarchies, active_categories):
            reciprocal_edges.append((u, v))
        else:
            secondary_lines.append(_get_line(subgraph, (u, v)))
    
    if debug:
        print(f"Edges: {len(reciprocal_edges)} reciprocal, {len(secondary_lines)} secondary")
        if active_hierarchies is not None or active_categories is not None:
            print(f"  Active filter: hierarchy={active_hierarchies}, category={active_categories}")
    
    # Find coplanar groups
    groups = find_coplanar_groups(
        subgraph,
        min_degree=min_degree,
        debug=debug,
        active_hierarchies=active_hierarchies,
        active_categories=active_categories,
    )
    
    if not groups:
        if debug:
            print("No coplanar groups - returning unshifted")
        shifted_lines = [_get_line(subgraph, e) for e in reciprocal_edges]
        if write_attributes:
            _store_shifted_lines(subgraph, reciprocal_edges, shifted_lines)
        return {
            'graph': subgraph,
            'shifted_lines': shifted_lines,
            'secondary_lines': secondary_lines,
            'nexus_lines': [],
            'coplanar_groups': [],
            'info': {'converged': True, 'cost': 0, 'n_vertices': 0}
        }
    
    # Track edges in groups
    edges_in_groups = set()
    for g in groups:
        for e in g['edges']:
            edges_in_groups.add(e)
            edges_in_groups.add((e[1], e[0]))
    
    unshifted = []
    shifted_by_edge = {}
    for e in reciprocal_edges:
        if e not in edges_in_groups:
            line = _get_line(subgraph, e)
            unshifted.append(line)
            shifted_by_edge[e] = line
    
    # Process each group
    shifted, nexus, infos = [], [], []
    total_verts = 0
    
    for gi, g in enumerate(groups):
        if debug:
            print(f"\n--- Group {gi}: {g['degree']} lines at node {g['node']} ---")

        # Rule 2: if nexus angle is too straight, do not reciprocalize at this group.
        if g.get('angles') and max(g['angles']) > straight_angle_threshold_deg:
            if debug:
                print(f"  skipped: max angle {max(g['angles']):.1f}° > {straight_angle_threshold_deg:.1f}°")
            shifted.extend(g['lines'])
            for edge, line in zip(g['edges'], g['lines']):
                shifted_by_edge[edge] = line
            continue

        # Rule 1: valency-2 special case before reciprocalization.
        # Keep one edge fixed and move the other away from the nexus by engage_len.
        if g['degree'] == 2 and len(g['lines']) == 2:
            fixed_line = g['lines'][0]
            moving_line = g['lines'][1]
            moving_end_index = _line_end_at_node(g['edges'][1], g['node'])
            moving_shifted = _offset_line_at_end(moving_line, moving_end_index, engage_len)
            shifted.extend([fixed_line, moving_shifted])
            shifted_by_edge[g['edges'][0]] = fixed_line
            shifted_by_edge[g['edges'][1]] = moving_shifted
            continue

        solver = ReciprocalSolver(g['lines'], engage_len=engage_len, tol=tol)
        
        if not solver.vertices:
            shifted.extend(g['lines'])
            for edge, line in zip(g['edges'], g['lines']):
                shifted_by_edge[edge] = line
            continue
        
        info = solver.solve(rotation_sign=rotation_sign)
        solver.trim_to_neighbors()
        total_verts += len(solver.vertices)
        
        if debug:
            print(f"  converged={info['converged']}, cost={info['cost']:.2e}")
        
        group_shifted = solver.to_lines()
        shifted.extend(group_shifted)
        for edge, line in zip(g['edges'], group_shifted):
            shifted_by_edge[edge] = line
        nexus.extend(solver.get_nexus_lines())
        infos.append(info)
    
    shifted_lines = shifted + unshifted
    if write_attributes:
        for edge, line in shifted_by_edge.items():
            subgraph.edge_attribute(edge, 'line', line)
            subgraph.edge_attribute(edge, 'shifted_lines', line)

    return {
        'graph': subgraph,
        'shifted_lines': shifted_lines,
        'secondary_lines': secondary_lines,
        'nexus_lines': nexus,
        'coplanar_groups': groups,
        'info': {
            'converged': all(i.get('converged', True) for i in infos),
            'cost': sum(i.get('cost', 0) for i in infos),
            'n_vertices': total_verts,
            'n_groups': len(groups),
        }
    }

def reciprocal_width_from_subgraph(subgraph, engage_len=0.11, tol=0.1, rotation_sign=+1,
                              min_degree=2, iterations=10, debug=True, straight_angle_threshold_deg=178.0,
                              active_hierarchies=None, active_categories=None,
                              write_attributes=True):
    """
    Reciprocalize lines using Global Relaxation to solve the see-saw effect,
    followed by a final cyclic intersection trimming pass.
    """
    global_beams = {}
    secondary_lines = []
    reciprocal_edges = []
    
    # 1. Collect edges and establish global beam tracking
    for u, v in subgraph.edges():
        if _is_reciprocal_edge(subgraph, (u, v), active_hierarchies, active_categories):
            reciprocal_edges.append((u, v))
            
            # Try to get 'width' from graph. Fallback to engage_len if None.
            w = subgraph.edge_attribute((u, v), 'width')
            shift_dist = (w / 2.0) if w else engage_len
            
            line = _get_line(subgraph, (u, v))
            p0, p1 = line.start, line.end
            
            global_beams[(u, v)] = {
                'p0': [p0.x, p0.y, p0.z], 
                'p1': [p1.x, p1.y, p1.z], 
                'shift': shift_dist
            }
        else:
            secondary_lines.append(_get_line(subgraph, (u, v)))
    
    if debug:
        print(f"Edges: {len(reciprocal_edges)} reciprocal, {len(secondary_lines)} secondary")
        if active_hierarchies is not None or active_categories is not None:
            print(f"  Active filter: hierarchy={active_hierarchies}, category={active_categories}")
    
    # 2. Find coplanar groups based on original unshifted topology
    groups = find_coplanar_groups(
        subgraph,
        min_degree=min_degree,
        debug=debug,
        active_hierarchies=active_hierarchies,
        active_categories=active_categories,
    )
    
    if not groups:
        shifted_lines = [_get_line(subgraph, e) for e in reciprocal_edges]
        if write_attributes:
            _store_shifted_lines(subgraph, reciprocal_edges, shifted_lines)
        return {
            'graph': subgraph,
            'shifted_lines': shifted_lines,
            'secondary_lines': secondary_lines,
            'nexus_lines': [], 'coplanar_groups': [],
            'info': {'converged': True, 'cost': 0, 'n_vertices': 0}
        }

    # 3. GLOBAL RELAXATION LOOP
    for iteration in range(iterations):
        global_intended_moves = {}
        
        for g in groups:
            # Rule 2: skip near-straight nexus groups.
            if g.get('angles') and max(g['angles']) > straight_angle_threshold_deg:
                continue

            # Rule 1: valency-2 special case before reciprocalization.
            if g['degree'] == 2 and len(g['edges']) == 2:
                moving_edge = g['edges'][1] if g['edges'][1] in global_beams else (g['edges'][1][1], g['edges'][1][0])
                node = g['node']
                b = global_beams[moving_edge]
                if moving_edge[0] == node:
                    d = subtract_vectors(b['p1'], b['p0'])
                    if length_vector(d) > 1e-9:
                        b['p0'] = add_vectors(b['p0'], scale_vector(normalize_vector(d), engage_len))
                else:
                    d = subtract_vectors(b['p0'], b['p1'])
                    if length_vector(d) > 1e-9:
                        b['p1'] = add_vectors(b['p1'], scale_vector(normalize_vector(d), engage_len))
                continue

            current_lines = []
            beam_mapping = [] 
            
            for e in g['edges']: 
                actual_edge = e if e in global_beams else (e[1], e[0])
                b = global_beams[actual_edge]
                
                is_same_dir = (e == actual_edge)
                p0, p1 = (b['p0'], b['p1']) if is_same_dir else (b['p1'], b['p0'])
                    
                current_lines.append(Line(Point(*p0), Point(*p1)))
                beam_mapping.append((actual_edge, is_same_dir))
                
            solver = ReciprocalSolver(current_lines, tol=tol)
            
            for local_idx, (actual_edge, _) in enumerate(beam_mapping):
                solver.beams[local_idx]['shift'] = global_beams[actual_edge]['shift']
                
            group_moves = solver.calculate_ideal_targets(rotation_sign)
            
            for (local_bi, local_ei), new_pt in group_moves.items():
                actual_edge, is_same_dir = beam_mapping[local_bi]
                global_ei = local_ei if is_same_dir else 1 - local_ei
                global_intended_moves[(actual_edge, global_ei)] = new_pt
                
        for (actual_edge, global_ei), new_pt in global_intended_moves.items():
            if global_ei == 0:
                global_beams[actual_edge]['p0'] = new_pt
            else:
                global_beams[actual_edge]['p1'] = new_pt

    # 3.5 POST-RELAXATION TRIMMING (Cyclic Intersection)
    # Now that all lines are settled, perfectly trim the tails at the intersections
    for g in groups:
        m = len(g['edges'])
        if m < 2:
            continue
            
        node = g['node'] # The central intersection node for this group
        
        # Pre-build Line objects for the current group state
        group_lines = []
        for e in g['edges']:
            actual_edge = e if e in global_beams else (e[1], e[0])
            b = global_beams[actual_edge]
            group_lines.append(Line(Point(*b['p0']), Point(*b['p1'])))
            
        # Intersect line k with the next line in the sequence
        for k in range(m):
            e_k = g['edges'][k]
            actual_edge_k = e_k if e_k in global_beams else (e_k[1], e_k[0])
            b_k = global_beams[actual_edge_k]
            
            # Determine the neighbor to trim against based on rotation direction
            target_k = (k + 1) % m if rotation_sign > 0 else (k - 1) % m
            
            line_a = group_lines[k]
            line_b = group_lines[target_k]
            
            # Find the 3D intersection. `intersection_line_line` returns a tuple of 
            # the two closest points on both lines. res[0] is the point on line_a.
            res = intersection_line_line(line_a, line_b)
            
            # Fallback for perfectly parallel lines (e.g., 180 deg splits)
            if not res:
                fallback_k = (k - 1) % m if rotation_sign > 0 else (k + 1) % m
                res = intersection_line_line(line_a, group_lines[fallback_k])
                
            if res:
                intersect_pt = res[0]
                
                # We update the endpoint that is touching the central node, cleanly cutting the tail
                if actual_edge_k[0] == node:
                    b_k['p0'] = [*intersect_pt]
                else:
                    b_k['p1'] = [*intersect_pt]

    # 4. Finalize output
    shifted_lines = []
    for e in reciprocal_edges:
        b = global_beams[e]
        shifted_line = Line(Point(*b['p0']), Point(*b['p1']))
        if write_attributes:
            subgraph.edge_attribute(e, "line", shifted_line)
            subgraph.edge_attribute(e, "shifted_lines", shifted_line)
        shifted_lines.append(shifted_line)
    
    return {
        'graph': subgraph,
        'shifted_lines': shifted_lines,
        'secondary_lines': secondary_lines,
        'nexus_lines': [], # Left empty, zero-gap means no central holes!
        'coplanar_groups': groups,
        'info': {
            'converged': True, 
            'cost': 0, 
            'iterations': iterations,
            'n_groups': len(groups),
        },
        "global lines": global_beams
    }


def reciprocal_orthogonal_from_subgraph(subgraph, **kwargs):
    """Run reciprocal only on v5 orthogonal primary edges."""
    return reciprocal_width_from_subgraph(
        subgraph,
        active_hierarchies='primary_orthogonal',
        active_categories='orthogonal',
        **kwargs
    )


def reciprocal_diagonal_from_subgraph(subgraph, **kwargs):
    """Run reciprocal only on v5 diagonal primary edges."""
    return reciprocal_width_from_subgraph(
        subgraph,
        active_hierarchies='primary_diagonal',
        active_categories=('default_diagonal', 'moved_diagonal'),
        **kwargs
    )
