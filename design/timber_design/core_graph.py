# -*- coding: utf-8 -*-
"""
Reciprocal Frame using COMPAS Graph

Beams = Graph nodes (with geometry attributes)
Connections = Graph edges (with constraint attributes)

"""

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

from compas.datastructures import Graph, Mesh
from compas.geometry import Line, Point, Plane, midpoint_line, intersection_line_plane


class ReciprocalFrame(Graph):
    """
    Reciprocal frame stored as COMPAS Graph.

    Nodes = beams (axes)
        - 'pts': np.array([start, end]) shape (2, 3)
        - 'faces': tuple (fkey1, fkey2) - the two mesh faces this beam connects

    Edges = connections (constraints)
        - 'face': fkey where connection occurs (knot)
        - 'ends': tuple (end_i, end_j) - which beam ends meet
        - 'xi': engagement parameter
    """

    @classmethod
    def from_mesh(cls, mesh):
        """Build reciprocal frame graph from triangulated COMPAS mesh.

        Concept:
        1. Every Mesh Edge -> 1 Beam (Node)
        2. Every Mesh Face -> Cycle of Connections (Edges)
        """
        rf = cls()
        rf.attributes["mesh"] = mesh

        # Pre-calculate centroids and xi values
        centroids = {f: Point(*mesh.face_centroid(f)) for f in mesh.faces()}
        xi_map = {f: mesh.face_attributes(f).get("xi", 0.2) for f in mesh.faces()}

        # Helper: Create or retrieve a beam for a given mesh edge
        edge_to_beam = {}

        def get_beam(u, v):
            # Sort vertex pair to ensure uniqueness (u,v) == (v,u)
            edge_key = tuple(sorted((u, v)))
            if edge_key in edge_to_beam:
                return edge_to_beam[edge_key]

            # Identify faces adjacent to this edge
            f1 = mesh.halfedge[u][v]
            f2 = mesh.halfedge[v][u]

            p1 = (
                centroids[f1]
                if f1 is not None
                else Point(
                    *midpoint_line(
                        (mesh.vertex_coordinates(u), mesh.vertex_coordinates(v))
                    )
                )
            )
            p2 = (
                centroids[f2]
                if f2 is not None
                else Point(
                    *midpoint_line(
                        (mesh.vertex_coordinates(u), mesh.vertex_coordinates(v))
                    )
                )
            )

            # Store Beam
            beam_line = Line(p1, p2)
            rf.add_node(edge_key, beam=beam_line)
            edge_to_beam[edge_key] = edge_key
            return edge_key

        # Main Loop: Iterate over faces to link beams
        for fkey in mesh.faces():
            # 1. Collect all beams around this face
            face_beams = []
            face_centroid = centroids[fkey]

            for u, v in mesh.face_halfedges(fkey):
                beam_id = get_beam(u, v)
                beam_line = rf.node_attribute(beam_id, "beam")

                # Determine which end of the beam corresponds to this face
                param = (
                    0 if beam_line.start.distance_to_point(face_centroid) < 1e-6 else 1
                )

                face_beams.append((beam_id, param))

            # 2. Connect them in a cycle
            # Beam i rests on Beam i+1, etc.
            n = len(face_beams)
            for i in range(n):
                (b1, t1) = face_beams[i]
                (b2, t2) = face_beams[(i + 1) % n]

                if not rf.has_edge(b1, b2):
                    rf.add_edge(b1, b2, connections=[])

                rf.edge_attribute((b1, b2), "connections").append(
                    {"ends": (t1, t2), "face": fkey, "xi": xi_map[fkey]}
                )

        return rf

    def get_beam(self, key):
        """Get beam as COMPAS Line."""
        return self.node_attribute(key, "beam")

    def get_beam_pts(self, key):
        """Get beam endpoints as numpy array."""
        line = self.node_attribute(key, "beam")
        return np.array([line.start, line.end])

    def set_beam_pts(self, key, pts):
        """Set beam endpoints."""
        line = self.node_attribute(key, "beam")
        line.start = Point(*pts[0])
        line.end = Point(*pts[1])

    def to_lines(self):
        """Export beams as COMPAS Lines."""
        return [self.node_attribute(key, "beam") for key in self.nodes()]

    def get_connection_lines(self):
        """Get eccentricity lines between connected beams.

        Computes the shortest distance line between each pair of connected
        beams using plane intersection method (works for skew lines).

        Returns: list of COMPAS Lines
        """
        connectors = []
        seen = set()

        for beam_i, end_i, beam_j, end_j, xi in self.get_connections():
            # Avoid duplicates for same beam pair
            pair_key = tuple(sorted((beam_i, beam_j)))
            if pair_key in seen:
                continue
            seen.add(pair_key)

            line_i = self.get_beam(beam_i)
            line_j = self.get_beam(beam_j)

            # Cross product gives direction perpendicular to both lines
            cross_p = line_i.direction.cross(line_j.direction)

            # Skip if lines are parallel (cross product is zero)
            if cross_p.length < 1e-10:
                continue

            plane_j = Plane(
                point=line_j.midpoint, normal=cross_p.cross(line_j.direction)
            )
            q1 = intersection_line_plane(line_i, plane_j)

            plane_i = Plane(
                point=line_i.midpoint, normal=cross_p.cross(line_i.direction)
            )
            q2 = intersection_line_plane(line_j, plane_i)

            if q1 is not None and q2 is not None:
                dist = Point(*q1).distance_to_point(Point(*q2))
                if dist > 1e-6:
                    connectors.append(Line(Point(*q1), Point(*q2)))

        return connectors

    def to_numpy(self):
        """Export all beam positions as single numpy array for optimization."""
        keys = list(self.nodes())
        n = len(keys)
        arr = np.zeros((n, 2, 3))
        for i, k in enumerate(keys):
            arr[i] = self.get_beam_pts(k)
        return arr, keys

    def from_numpy(self, arr, keys):
        """Import beam positions from numpy array."""
        for i, k in enumerate(keys):
            self.set_beam_pts(k, arr[i])

    def get_connections(self):
        """Yield all connections as (beam_i, end_i, beam_j, end_j, xi)."""
        for u, v in self.edges():
            for conn in self.edge_attribute((u, v), "connections"):
                end_i, end_j = conn["ends"]
                yield u, end_i, v, end_j, conn["xi"]

    def _build_connection_arrays(self, keys):
        """Build vectorized index arrays for fast residual computation."""
        key_to_idx = {k: i for i, k in enumerate(keys)}

        i_idx, end_i, j_idx, end_j, xi_arr = [], [], [], [], []
        for beam_i, ei, beam_j, ej, xi in self.get_connections():
            i_idx.append(key_to_idx[beam_i])
            end_i.append(ei)
            j_idx.append(key_to_idx[beam_j])
            end_j.append(ej)
            xi_arr.append(xi)

        return (
            np.array(i_idx, dtype=np.int32),
            np.array(end_i, dtype=np.int32),
            np.array(j_idx, dtype=np.int32),
            np.array(end_j, dtype=np.int32),
            np.array(xi_arr),
        )

    def _build_sparsity(
        self, n_beams, n_conn, n_res, i_idx, j_idx, wt, we, wf, ws, has_srf
    ):
        """Build Jacobian sparsity pattern for efficient finite differences."""
        n_vars = n_beams * 6
        sparsity = lil_matrix((n_res, n_vars), dtype=np.int8)

        row = 0

        # wt residuals: each connection depends on beam i and beam j (12 vars per 3 residuals)
        if wt > 0:
            for k in range(n_conn):
                i, j = i_idx[k], j_idx[k]
                cols_i = list(range(i * 6, i * 6 + 6))
                cols_j = list(range(j * 6, j * 6 + 6))
                for r in range(3):
                    sparsity[row + r, cols_i] = 1
                    sparsity[row + r, cols_j] = 1
                row += 3

        # we residuals: each connection depends on beam i and beam j (12 vars per 1 residual)
        if we > 0:
            for k in range(n_conn):
                i, j = i_idx[k], j_idx[k]
                cols_i = list(range(i * 6, i * 6 + 6))
                cols_j = list(range(j * 6, j * 6 + 6))
                sparsity[row, cols_i] = 1
                sparsity[row, cols_j] = 1
                row += 1

        # wf residuals: each beam depends only on itself (6 vars per 6 residuals)
        if wf > 0:
            for i in range(n_beams):
                cols = list(range(i * 6, i * 6 + 6))
                for r in range(6):
                    sparsity[row + r, cols[r]] = 1
                row += 6

        # ws residuals: z coord depends only on itself
        if ws > 0 and has_srf:
            for i in range(n_beams):
                # endpoint 0 z: var index i*6 + 2
                # endpoint 1 z: var index i*6 + 5
                sparsity[row, i * 6 + 2] = 1
                sparsity[row + 1, i * 6 + 5] = 1
                row += 2

        return sparsity.tocsr()

    def solve(
        self, weights=(1.0, 0.0, 0.1, 0.0), eccentricity=0.00, srf=None, use_sparse=True
    ):
        """
        Solve reciprocal frame using scipy least_squares (vectorized).

        weights: (wt, we, wf, ws)
            wt = reciprocal target
            we = eccentricity (min beam separation)
            wf = fidelity to original
            ws = surface projection
        eccentricity: float - minimum separation distance for eccentricity constraint
        use_sparse: bool - use sparse Jacobian (faster for large meshes)

        Returns: info dict
        """
        wt, we, wf, ws = weights

        # Flatten to numpy
        x0, keys = self.to_numpy()
        original = x0.flatten()
        n_beams = len(keys)

        # Pre-build connection index arrays (done once)
        i_idx, end_i, j_idx, end_j, xi_arr = self._build_connection_arrays(keys)
        n_conn = len(i_idx)
        t_arr = xi_arr / 2.0
        other_end_j = 1 - end_j

        # Pre-calculate residual size
        n_res = 0
        if wt > 0:
            n_res += n_conn * 3
        if we > 0:
            n_res += n_conn
        if wf > 0:
            n_res += n_beams * 6
        if ws > 0 and srf is not None:
            n_res += n_beams * 2

        srf_val = (
            float(srf) if srf is not None and isinstance(srf, (int, float)) else 0.0
        )
        has_srf = srf is not None and isinstance(srf, (int, float))

        def residuals(x):
            pts = x.reshape(n_beams, 2, 3)
            res = np.zeros(n_res)
            idx = 0

            if wt > 0:
                p_ref = pts[j_idx, end_j]
                p_other = pts[j_idx, other_end_j]
                targets = p_ref + t_arr[:, None] * (p_other - p_ref)
                current = pts[i_idx, end_i]
                res[idx : idx + n_conn * 3] = ((current - targets) * wt).flatten()
                idx += n_conn * 3

            if we > 0:
                p = pts[i_idx, end_i]
                a, b = pts[j_idx, 0], pts[j_idx, 1]
                ab = b - a
                ab_dot = np.sum(ab * ab, axis=1) + 1e-10
                t_proj = np.clip(np.sum((p - a) * ab, axis=1) / ab_dot, 0, 1)
                dist = np.linalg.norm(p - (a + t_proj[:, None] * ab), axis=1)
                res[idx : idx + n_conn] = np.maximum(0, eccentricity - dist) * we
                idx += n_conn

            if wf > 0:
                res[idx : idx + n_beams * 6] = (x - original) * wf
                idx += n_beams * 6

            if ws > 0 and has_srf:
                res[idx : idx + n_beams * 2] = (pts[:, :, 2].flatten() - srf_val) * ws

            return res

        # Build sparsity pattern if enabled
        if use_sparse and n_beams > 50:
            sparsity = self._build_sparsity(
                n_beams, n_conn, n_res, i_idx, j_idx, wt, we, wf, ws, has_srf
            )
            result = least_squares(
                residuals,
                x0.flatten(),
                method="trf",
                jac_sparsity=sparsity,
                ftol=1e-8,
                xtol=1e-8,
                max_nfev=5000,
            )
        else:
            result = least_squares(
                residuals,
                x0.flatten(),
                method="lm",
                ftol=1e-8,
                xtol=1e-8,
                max_nfev=5000,
            )

        # Update graph
        self.from_numpy(result.x.reshape(n_beams, 2, 3), keys)

        return {
            "converged": result.success,
            "nfev": result.nfev,  # function evaluations
            "residual": float(result.cost),
            "message": result.message,
            "sparse": use_sparse and n_beams > 50,
        }
