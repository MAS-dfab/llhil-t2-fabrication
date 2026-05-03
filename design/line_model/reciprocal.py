# -*- coding: utf-8 -*-
"""
Reciprocal Frame Solver

Usage:
    from design.line_model.reciprocal import reciprocal_from_subgraph
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
    line = subgraph.edge_attribute((u, v), 'line')
    if line and isinstance(line, Line):
        return line
    if line:
        return Line(Point(*line[0]), Point(*line[1]))
    return Line(_get_pt(subgraph, u), _get_pt(subgraph, v))


# ==============================================================================
# Coplanar Group Detection
# ==============================================================================

def find_coplanar_groups(subgraph, min_degree=2, plane_tol=0.1, debug=True):
    """
    Find groups of primary+double edges that lie on the same plane.
    Returns groups where 2+ edges meet at a joint.
    """
    from compas.geometry import Frame
    
    # Get all primary AND double edges
    edges = [(u, v) for u, v in subgraph.edges() 
             if subgraph.edge_attribute((u, v), 'hierarchy') in ('primary', 'double')]
    
    if debug:
        print(f"Total primary+double edges: {len(edges)}")
    
    if not edges:
        return []
    
    def edge_midpoint(edge):
        p1, p2 = _get_pt(subgraph, edge[0]), _get_pt(subgraph, edge[1])
        return Point((p1.x + p2.x)/2, (p1.y + p2.y)/2, (p1.z + p2.z)/2)
    
    def edge_direction(edge):
        p1, p2 = _get_pt(subgraph, edge[0]), _get_pt(subgraph, edge[1])
        d = Vector(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
        d.unitize()
        return d
    
    def make_plane_frame(edge):
        """Create a plane frame from edge: origin=midpoint, normal=edge×Z"""
        mid = edge_midpoint(edge)
        d = edge_direction(edge)
        
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
        p1, p2 = _get_pt(subgraph, edge[0]), _get_pt(subgraph, edge[1])
        local1 = frame.to_local_coordinates(p1)
        local2 = frame.to_local_coordinates(p2)
        return abs(local1[2]) < tol and abs(local2[2]) < tol
    
    # Group edges by plane
    plane_groups = []
    remaining = list(edges)
    
    while remaining:
        # Pick first edge to define a plane
        seed = remaining.pop(0)
        frame, normal = make_plane_frame(seed)
        
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
            dirs = []
            for e in node_edge_list:
                other = e[1] if e[0] == node else e[0]
                other_pt = _get_pt(subgraph, other)
                d = Vector(other_pt.x - joint_pt.x, 
                          other_pt.y - joint_pt.y, 
                          other_pt.z - joint_pt.z)
                d.unitize()
                dirs.append(d)
            
            # Cyclic sort
            sorted_edges, sorted_dirs = _cyclic_sort(node_edge_list, dirs, normal)
            angles = _compute_angles(sorted_dirs, normal)
            
            if debug:
                print(f"\n  Plane {plane_idx}, Joint {node}: {len(node_edge_list)} edges")
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
                'degree': len(node_edge_list),
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
                              min_degree=2, debug=True, straight_angle_threshold_deg=178.0):
    """
    Reciprocalize primary+double lines from a classified subgraph.
    
    Each coplanar group is processed SEPARATELY so vertices shared between
    groups stay FIXED unless they have 3+ edges within the SAME group.
    
    Returns dict with: shifted_lines, secondary_lines, nexus_lines, coplanar_groups, info
    """
    # Collect edges by type
    reciprocal_edges = []
    secondary_lines = []
    
    for u, v in subgraph.edges():
        etype = subgraph.edge_attribute((u, v), 'hierarchy')
        if etype in ('primary', 'double'):
            reciprocal_edges.append((u, v))
        else:
            secondary_lines.append(_get_line(subgraph, (u, v)))
    
    if debug:
        print(f"Edges: {len(reciprocal_edges)} reciprocal, {len(secondary_lines)} secondary")
    
    # Find coplanar groups
    groups = find_coplanar_groups(subgraph, min_degree=min_degree, debug=debug)
    
    if not groups:
        if debug:
            print("No coplanar groups - returning unshifted")
        return {
            'shifted_lines': [_get_line(subgraph, e) for e in reciprocal_edges],
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
    
    unshifted = [_get_line(subgraph, e) for e in reciprocal_edges if e not in edges_in_groups]
    
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
            continue

        # Rule 1: valency-2 special case before reciprocalization.
        # Keep one edge fixed and move the other away from the nexus by engage_len.
        if g['degree'] == 2 and len(g['lines']) == 2:
            fixed_line = g['lines'][0]
            moving_line = g['lines'][1]
            moving_end_index = _line_end_at_node(g['edges'][1], g['node'])
            shifted.extend([fixed_line, _offset_line_at_end(moving_line, moving_end_index, engage_len)])
            continue

        solver = ReciprocalSolver(g['lines'], engage_len=engage_len, tol=tol)
        
        if not solver.vertices:
            shifted.extend(g['lines'])
            continue
        
        info = solver.solve(rotation_sign=rotation_sign)
        solver.trim_to_neighbors()
        total_verts += len(solver.vertices)
        
        if debug:
            print(f"  converged={info['converged']}, cost={info['cost']:.2e}")
        
        shifted.extend(solver.to_lines())
        nexus.extend(solver.get_nexus_lines())
        infos.append(info)
    
    return {
        'shifted_lines': shifted + unshifted,
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
                              min_degree=2, iterations=10, debug=True, straight_angle_threshold_deg=178.0):
    """
    Reciprocalize lines using Global Relaxation to solve the see-saw effect,
    followed by a final cyclic intersection trimming pass.
    """
    global_beams = {}
    secondary_lines = []
    reciprocal_edges = []
    
    # 1. Collect edges and establish global beam tracking
    for u, v in subgraph.edges():
        # print(subgraph.edge_attribute((u, v), "hierarchy"))
        etype = subgraph.edge_attribute((u, v), 'hierarchy')
        if etype in ('primary', 'double'):
            # print("IN")
            reciprocal_edges.append((u, v))
            
            # Try to get 'width' from graph. Fallback to engage_len if None.
            w = subgraph.edge_attribute((u, v), 'width')
            shift_dist = (w / 2.0) if w else engage_len
            
            p0, p1 = _get_pt(subgraph, u), _get_pt(subgraph, v)
            
            global_beams[(u, v)] = {
                'p0': [p0.x, p0.y, p0.z], 
                'p1': [p1.x, p1.y, p1.z], 
                'shift': shift_dist
            }
        else:
            secondary_lines.append(_get_line(subgraph, (u, v)))
    
    if debug:
        print(f"Edges: {len(reciprocal_edges)} reciprocal, {len(secondary_lines)} secondary")
    
    # 2. Find coplanar groups based on original unshifted topology
    groups = find_coplanar_groups(subgraph, min_degree=min_degree, debug=debug)
    
    if not groups:
        return {
            'shifted_lines': [_get_line(subgraph, e) for e in reciprocal_edges],
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
        subgraph.edge_attribute(e, "shifted_lines", shifted_line)
        shifted_lines.append(shifted_line)
    
    return {
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
