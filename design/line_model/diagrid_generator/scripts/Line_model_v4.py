### Edited by Jerry on 27 Mar 2026
from Structs_v1 import Pair, BeamCategory, Beam, Transform, CrossSection

from compas.geometry import Line, Vector, Box, Point, Plane, Polyline, Polygon
from compas.geometry import is_point_in_polygon_xy, is_point_on_polyline_xy, intersection_line_triangle
import math

########-------Helpers---------############
def remap_number(value, from_min, from_max, to_min=0, to_max=1):
    return to_min + (to_max - to_min) * ((value - from_min) / (from_max - from_min))

def divide_by_count(line, count):
    pts = []
    for i in range(count):
        t = i / count
        pt = line.point_at(t)
        pts.append(pt)
    pts.append(line.point_at(1))

    return pts


def average_points(points):
    if not points:
        return None
    x = sum(point.x for point in points) / len(points)
    y = sum(point.y for point in points) / len(points)
    z = sum(point.z for point in points) / len(points)

    return Point(x, y, z)


def compute_aabb_xy(polyline):
    if not isinstance(polyline, Polyline):
        raise ValueError("polyline must be compas Polyline.")
    
    x_coords = [p.x for p in polyline]
    y_coords = [p.y for p in polyline]
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)

    return ((min_x, min_y), (max_x, max_y))


def is_point_in_aabb_xy(point, aabb, tol=1e-3):
    """
    Test if a point is inside an axis-aligned bounding box (AABB) in the XY plane.
    Also consider points that are on the boundary of the AABB as inside.

    Args:
        point (Point):
        aabb (two tuples):
    """
    (min_x, min_y), (max_x, max_y) = aabb
    
    return min_x - tol <= point.x <= max_x + tol and min_y - tol <= point.y <= max_y + tol


##########-----------Class--------###########
class VertexList:
    def __init__(self, boundary, division_x=4, division_y=6, height=8.6):
        """
        A list of points representing the nodes.

        Args:
            boundary (Polyline): A closed polyline representing the boundary of the diagrid.
            division_x (int): Number of divisions along the x-axis.
            division_y (int): Number of divisions along the y-axis.
            height (float): The total height of the diagrid structure.

            vertices (list of Point): The list of vertices representing the nodes of the diagrid.
            pairs (list of tuple): (start_idx, end_idx) representing the edges between the vertices.
        """
        self.boundary = boundary
        self.div_x = division_x
        self.div_y = division_y
        self.height = height

        self.vertices = []
        self.pairs = []
        self.skip_indices = []

        self.dir1 = None
        self.dir2 = None
        self.height_list = []
        self.roof = None

        self.default_z = []
        self._default_vertices = []
        self._default_pairs = []

        self._cached_topology = None
        

    def __getitem__(self, idx):
        return self.vertices[idx]

    def __setitem__(self, idx, value):
        self.vertices[idx] = value

    def __len__(self):
        return len(self.vertices)
    
    def __iter__(self):
        return iter(self.vertices)


    def reset_default(self):
        self.vertices = self._default_vertices[:]
        self.pairs = self._default_pairs[:]


    def add_vertices(self, vertices):
        if not isinstance(vertices, list):
            vertices = [vertices]

        for vertex in vertices:
            if not isinstance(vertex, Point):
                raise ValueError("Only compas Point instances can be added to VertexList.")
            
            self.vertices = self.vertices + [vertex]


    def add_pairs(self, pairs):
        if not isinstance(pairs, list):
            pairs = [pairs]

        for pair in pairs:
            # Change this:
            # if not isinstance(pair, Pair):
            #     raise ValueError("Pairs must be instances of the Pair class.")
            
            # To this:
            if type(pair).__name__ != "Pair":
                raise ValueError("Pairs must be instances of the Pair class.")
            self.pairs.append(pair)

        self._cached_topology = None


    def delete_pairs(self, tuples):
        """
        Delete pairs by a list of tuples (start_idx, end_idx). You do not need to consider CrossSection and BeamCategory.
        """

        if not isinstance(tuples, list):
            tuples = [tuples]

        for pair in self.pairs:
            tuple_pair = (pair.start_idx, pair.end_idx)
            if tuple_pair in tuples:
                self.pairs.remove(pair)

        self._cached_topology = None

        
    def skip(self, indices):
        """
        Remove edges that are connected to the specified vertex indices.
        
        Args:
            indices (int or list of int): The vertex indices to skip. Can be a single index or a list of indices.
        """

        if not isinstance(indices, list):
            indices = [indices]
        
        for idx in indices:
            if idx in self.skip_indices:
                continue
            self.skip_indices.append(int(idx))

        for idx in self.skip_indices:
            self.pairs = [pair for pair in self.pairs if idx not in pair]

        self._cached_topology = None


    def move(self, indices, vectors):
        if isinstance(indices, list) and not isinstance(vectors, list):
            vectors = [vectors] * len(indices)

        if not isinstance(indices, list):
            indices = [indices]
            vectors = [vectors]
        for idx, vector in zip(indices, vectors):
            self.vertices[idx] += vector


    def compute_diagrid(self):
        """
        Compute the vertices and edge pairs for a diagrid structure based on the given boundary and divisions.

        Returns:
            vertices (list of Point): The computed vertices of the diagrid.
            pairs (list of tuple): The computed edge pairs representing the connections between the vertices.
        """

        if self.vertices:
            raise ValueError("Vertices already exist in the graph. Clear the graph before computing a new diagrid.")

        boundary = self.boundary
        division_x = self.div_x
        division_y = self.div_y

        # 1. Populate vertices of the diagrid
        vec_x = boundary[1] - boundary[0]
        vec_y = boundary[-2] - boundary[0]
        vec_z = vec_x.cross(vec_y)
        vec_x.unitize()
        vec_y.unitize()
        vec_z.unitize()
        
        dist_x = boundary[0].distance_to_point(boundary[1])
        dist_y = boundary[0].distance_to_point(boundary[-2])
        step_x = dist_x / division_x
        step_y = dist_y / division_y


        start = boundary[0].copy()
        p1 = start + (vec_x * step_x) + (vec_y * step_y)
        p2 = start + (vec_x * step_x)
        p3 = start + (vec_y * step_y)
        self.dir1 = p1 - start
        self.dir2 = p3 - p2

        div_x = division_x + 1
        div_y = division_y + 1
        div_z = division_x + 1
        for i in range(div_z):  # how many levels of points in z direction
            for j in range(div_x):
                for k in range(div_y):
                    p = start + (vec_x * j * step_x) + (vec_y * k * step_y)
                    p.name = f"lvl_{i}"
                    
                    self.vertices.append(p)
                    self.default_z.append(p.z)

            start += vec_x * step_x / 2
            start += vec_y * step_y / 2

            div_x -= 1
            div_y -= 1
        

        # 2. Find indices of start and end points of the diagrid edges
        self.pairs = []
        # Set default cross section sizes by levels
        crosec_list = CrossSection.ALL  # Assign smallest cross section to the top level and largest to the bottom level

        start_indices = []
        curr_idx = 0
        div_x = division_x + 1
        div_y = division_y + 1
        div_z = division_x + 1
        for i in range(div_z):
            start_indices.append(curr_idx)
            curr_idx += div_x * div_y
            div_x -= 1
            div_y -= 1

        curr_div_x = division_x + 1
        curr_div_y = division_y + 1
        div_z = division_x
        for i in range(div_z):
            curr_level_start = start_indices[i]
            next_level_start = start_indices[i+1]

            next_div_x = curr_div_x - 1
            next_div_y = curr_div_y - 1
            for j in range(next_div_x):
                for k in range(next_div_y):
                    target_idx = next_level_start + ((j * next_div_y) + k)

                    # (j, k), (j, k+1), (j+1, k+1), (j+1, k)
                    idx1 = curr_level_start + (j * curr_div_y + k)
                    idx2 = curr_level_start + (j * curr_div_y + (k+1))
                    idx3 = curr_level_start + ((j+1) * curr_div_y + (k+1))
                    idx4 = curr_level_start + ((j+1) * curr_div_y + k)

                    self.pairs.append(Pair(idx1, target_idx, level=i, cross_section=crosec_list[i], categories=[BeamCategory.SINGLE]))
                    self.pairs.append(Pair(idx2, target_idx, level=i, cross_section=crosec_list[i], categories=[BeamCategory.SINGLE]))
                    self.pairs.append(Pair(idx3, target_idx, level=i, cross_section=crosec_list[i], categories=[BeamCategory.SINGLE]))
                    self.pairs.append(Pair(idx4, target_idx, level=i, cross_section=crosec_list[i], categories=[BeamCategory.SINGLE]))
            curr_div_x -= 1
            curr_div_y -= 1
        

        # 3. Find pairs for adding edge beams
        max_indices = (self.div_x + 1) * (self.div_y + 1)
        facade_indices_a = list(range(0, max_indices, self.div_y + 1))
        facade_indices_b = list(range(self.div_y, max_indices, self.div_y + 1))
        
        for i in range(len(facade_indices_a) - 1):
            self.pairs.append(Pair(facade_indices_a[i], facade_indices_a[i+1], level=0, cross_section=CrossSection.M, categories=[BeamCategory.EDGE]))
            self.pairs.append(Pair(facade_indices_b[i], facade_indices_b[i+1], level=0, cross_section=CrossSection.M, categories=[BeamCategory.EDGE]))


        self._default_vertices = self.vertices
        self._default_pairs = self.pairs
        return self.vertices, self.pairs


    def set_default_height(self, height_list):
        """
        Create the structure profile by assigning a list of Z values to different levels of vertices.
        
        Args:
            height_list (list of float)(optional): A list of heights representing the point.z values for each level. If not provided, the height will be divided equally. The length of height_list should be equal to (division_x + 1) if provided.

        """
        if not self.vertices:
            raise ValueError("Vertices have not been computed. Call compute_diagrid first.")
        
        self.height_list = []
        div_z = self.div_x + 1

        if not height_list:
            height_list = [self.height / self.div_x * i for i in range(div_z)]

        if len(height_list) != div_z:
            raise ValueError(f"Height list length must be equal to division_z ({div_z}).")
        
        self.height_list = height_list

        for idx, vertex in enumerate(self._default_vertices):
            if vertex is None:
                continue
            vertex.z = self.default_z[idx] - height_list[int(vertex.name.split('_')[1])]


    def set_default_roof(self, mesh):
        """
        Project the highest level of vertices to a given mesh.

        Args:
            mesh (Mesh): representing the CLT panels on the roof.
        """
        if not self._default_vertices:
            raise ValueError("Vertices have not been computed. Call compute_diagrid first.")
        
        vertices, faces = mesh.to_vertices_and_faces()
        for idx, vertex in enumerate(self._default_vertices):
            line = Line(vertex, vertex + Vector(0, 0, 1))

            for face in faces:
                tri = [vertices[f] for f in face]
                result = intersection_line_triangle(line, tri)
                if result:
                    hit = Point(result[0], result[1], result[2])
                    
                    vertex.z = hit.z
                    self.default_z[idx] = hit.z
                    break
        self.roof = mesh


    @property
    def facade_points(self):
        if not self.vertices:
            raise ValueError("Vertices have not been computed. Call compute_diagrid first.")
        
        max_indices = (self.div_x + 1) * (self.div_y + 1)
        facade_indices = list(range(0, max_indices, self.div_y + 1)) + list(range(self.div_y, max_indices, self.div_y + 1))
        # print(f"Facade indices: {facade_indices}")
        return [self.vertices[idx] for idx in facade_indices]
    
    @property
    def roof_points(self):
        if not self.vertices:
            raise ValueError("Vertices have not been computed. Call compute_diagrid first.")
        
        max_indices = (self.div_x + 1) * (self.div_y + 1)
        return self.vertices[:max_indices]
    
    @property
    def support_points(self):
        if not self.vertices:
            raise ValueError("Vertices have not been computed. Call compute_diagrid first.")
        
        return [self._default_vertices[-1], self._default_vertices[-3]]
        
    
    def group_by_diamond(self):
        """
        Create a dictionary grouping vertices.

        Returns:
            groups (dict): {root0, [idx1, idx2, idx3, idx4], root1, [idx1, idx2, idx3, idx4], ...}
        """

        if self.div_x is None or self.div_y is None:
            raise ValueError("compute_diagrid must be called before grouping vertices.")
        
        dx = self.div_x + 1
        dy = self.div_y + 1
        dz = self.div_x + 1

        total = sum((dx - i) * (dy - i) for i in range(dz))
        roots = list(range(dx * dy, total))

        groups = {}
        for root in roots:
            for pair in self.pairs:
                if root == pair[1]:
                    groups.setdefault(root, []).append(pair[0])
        return groups

    
    def group_by_side(self):
        center = average_points(self.boundary[:-1])
        vec_y = self.boundary[-2] - self.boundary[0]
        test_pln = Plane(center, vec_y)
        
        dx = self.div_x + 1
        dy = self.div_y + 1
        dz = self.div_x + 1
        total = sum((dx - i) * (dy - i) for i in range(dz))

        groups = {}
        for i in range(total):
            pt = self.vertices[i]
            dist = pt.distance_to_plane(test_pln)
            groups.setdefault(dist, []).append(i)

        sorted_groups = dict(sorted(groups.items()))
        return sorted_groups


    @property
    def topology(self):
        """
        A dictionary representing the vertex and its connected neighbors.
        {vertex_idx: [neighbor_idx1, neighbor_idx2, ...], ...}
        """

        if self._cached_topology is not None:
            return self._cached_topology

        topology = {}
        for start, end in self.pairs:
            topology.setdefault(start, []).append(end)
            topology.setdefault(end, []).append(start)
        

        self._cached_topology = dict(sorted(topology.items()))
        return self._cached_topology
    

    @property
    def valency_map(self):
        """
        A dictionary representing the valency and the corresponding vertex indices.
        {valency: [vertex_idx1, vertex_idx2, ...], ...}
        """

        v_map = {}
        for vertex_idx, neighbors in self.topology.items():
            v_map.setdefault(len(neighbors), []).append(vertex_idx)

        v_map = dict(sorted(v_map.items()))
        return v_map
    

    def group_by_valency(self, valency):
        if valency not in self.valency_map.keys():
            raise ValueError(f"valency {valency} does not exist, Available valencies: {list(self.valency_map.keys())}")

        groups = {}

        nodes = self.valency_map.get(valency)
        for parent in nodes:
            children = self.topology.get(parent)
            groups.setdefault(parent, children)

        return groups


    def transform(self, valency):
        groups = self.group_by_valency(valency)
        return Transform(valency, groups)






class BeamList:
    def __init__(self, vertex_list):
        self.v_list = vertex_list
        self.pairs = vertex_list.pairs
        self.beams = []

        self.roof = vertex_list.roof
        self.clt_panels = []
        
        self.group = {}

    def set_default_beams(self):
        self.beams = []

        for pair in self.pairs:
            beam = Beam(pair, self.v_list)

            self.beams.append(beam)


    def __getitem__(self, idx):
        return self.beams[idx]
    
    def __setitem__(self, idx, value):
        self.beams[idx] = value

    def __len__(self):
        return len(self.beams)
    
    def __iter__(self):
        return iter(self.beams)
    
    @property
    def axises(self):
        return [beam.axis if beam is not None else None for beam in self.beams]
    

    def double(self, indices):
        
        for idx in indices:
            curr_beam = self.beams[idx]

            if curr_beam is None:
                continue

            if BeamCategory.SINGLE in curr_beam.categories:
                curr_beam.categories.remove(BeamCategory.SINGLE)

            # 1. Delete current beam
            self.v_list.delete_pairs((curr_beam.start_idx, curr_beam.end_idx))
            self.v_list.skip_indices.append(curr_beam.start_idx)
            self.v_list.skip_indices.append(curr_beam.end_idx)

            # 2. Offset beam on both sides
            offset = curr_beam.width * 0.001 / 2 if curr_beam.width is not None else CrossSection.XS[0] * 0.002 / 2

            offset_vec = curr_beam.direction.cross(Vector(0, 0, 1))
            offset_vec.unitize()

            for side in [-1, 1]:
                new_start = curr_beam.start.translated(offset_vec * offset * side)
                new_end = curr_beam.end.translated(offset_vec * offset * side)
                new_start.name = curr_beam.start.name
                new_end.name = curr_beam.end.name

                # 3. Add new pair
                curr_idx = len(self.v_list)
                new_pair = Pair(curr_idx, curr_idx + 1, level=curr_beam.pair.level, cross_section=curr_beam.cross_section, categories=[BeamCategory.DOUBLE])
                new_pair.categories.extend(curr_beam.categories)
                
                self.v_list.add_pairs(new_pair)

                # 4. Add four new vertices
                self.v_list.add_vertices(new_start)
                self.v_list.add_vertices(new_end)
                
                self.beams.append(Beam(new_pair, self.v_list))
            
        for idx in indices:
            self.beams[idx] = None


    def group_by_levels(self, level_list=[2, 1, 1], tol=1e-3):
        """
        Args:
            level_list (list of int): A list of integers partitioning the beams into different levels (start from top).
        """
        height_list = self.v_list.height_list
        height_list = [self.v_list.boundary[0].z - h for h in height_list]

        if sum(level_list) != len(height_list) - 1:
            raise ValueError("length of level_list must be equal to len(height_list) - 1.")
        
        intervals = []
        curr = 0
        for lnl in level_list:
            high = height_list[curr]
            low = height_list[curr + lnl]
            intervals.append((high, low))
            curr += lnl

        print (f"Height list: {height_list}")
        print (f"Intervals: {intervals}")

        group = {}
        for idx, beam in enumerate(self.beams):
            if beam is None:
                continue
            mid_z = beam.mid.z
            for i, (high, low) in enumerate(intervals):
                if low - tol < mid_z < high + tol:
                    group.setdefault(i, []).append(idx)
                    break
        return intervals, group
    


    def group_by_module(self, polylines, tol=1e-3):
        """
        Group beams based on the CLT panels and different levels of the diagrid
        
        Args:
            polylines (list of Polyline): representing the boundaries of CLT panels.
            
        Returns:
            group (dict): {panel_idx: [beam_idx1, beam_idx2, ...], ...}
        """
        self.clt_panels = polylines

        # 1. Find if point on or in polylines
        on_list = []  # list of points
        in_list = []
        for i, beam in enumerate(self.beams):
            if beam is None:
                continue
            p = beam.mid
            p.name = str(i)
            for poly in polylines:
                # Filter polylines by AABB
                aabb = compute_aabb_xy(poly)
                if not is_point_in_aabb_xy(p, aabb, tol):
                    continue

                is_on = is_point_on_polyline_xy(p, poly, tol)
                if is_on:
                    on_list.append(p)
                    break
            
            if p not in on_list:
                in_list.append(p)

        for i, poly in enumerate(polylines):
            poly.name = str(i)
        # return [p.name for p in on_list], [p.name for p in in_list]
            
        # 2. Find which panel belongs to point
        def belong(point, polylines):
            two_panels = []

            for idx, polyline in enumerate(polylines):
                aabb = compute_aabb_xy(polyline)
                if not is_point_in_aabb_xy(point, aabb, tol):
                    continue

                if is_point_on_polyline_xy(point, polyline, tol):
                    two_panels.append(polyline)
            
            best = max(two_panels, key=lambda p: Polygon(list(p)).area)
            return best
    
        self.group = {}
        for p in on_list:
            polyline = belong(p, polylines)
            if polyline is None:
                continue
            self.group.setdefault(int(polyline.name), []).append(int(p.name))


        for p in in_list:
            for poly in polylines:

                if is_point_in_polygon_xy(p, Polygon(list(poly))):
                    
                    self.group.setdefault(int(poly.name), []).append(int(p.name))
                    break

        self.group = dict(sorted(self.group.items()))
        return self.group
    

    def group_by_cross_section(self):
        pass