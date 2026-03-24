### Edited by Jerry on 24 Mar 2026

from compas.geometry import Line, Vector, Box, Point, Plane
from compas.geometry import is_point_in_polygon_xy, intersection_line_plane
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
            height_list (list of float)(optional): A list of heights representing the point.z values for each level. If not provided, the height will be divided equally. The length of height_list should be equal to (division_x + 1) if provided.

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
        self.default_z = []

        self._default_vertices = []
        self._default_pairs = []

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
            
            # self.vertices.append(vertex)
            self.vertices = self.vertices + [vertex]


    def add_pairs(self, pairs):
        if not isinstance(pairs, list):
            pairs = [pairs]

        for pair in pairs:
            if not isinstance(pair, tuple) or len(pair) != 2:
                raise ValueError("Pairs must be tuples of two vertex indices.")
            
            self.pairs.append(pair)
            

    def skip(self, indices):
        """
        Remove edges that are connected to the specified vertex indices.
        
        Args:
            indices (int or list of int): The vertex indices to skip. Can be a single index or a list of indices.
        """
        self.skip_indices = []

        if not isinstance(indices, list):
            indices = [indices]
        self.skip_indices.extend(indices)
        
        for idx in indices:
            self.pairs = [pair for pair in self.pairs if idx not in pair]
            # self.pairs[idx] = (None, None)


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
                    p.name = f"pt_{i}_{j}_{k}"
                    
                    self.vertices.append(p)
                    self.default_z.append(p.z)

            start += vec_x * step_x / 2
            start += vec_y * step_y / 2

            div_x -= 1
            div_y -= 1
        

        # 2. Find indices of start and end points of the diagrid edges
        self.pairs = []

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

                    self.pairs.append((idx1, target_idx))
                    self.pairs.append((idx2, target_idx))
                    self.pairs.append((idx3, target_idx))
                    self.pairs.append((idx4, target_idx))
            curr_div_x -= 1
            curr_div_y -= 1
        
        self._default_vertices = self.vertices
        self._default_pairs = self.pairs
        return self.vertices, self.pairs


    def set_default_height(self, height_list):
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
            # vertex.z -= height_list[int(vertex.name.split('_')[1])]


    def deform_roof(self, polygons):
        if not self.vertices:
            raise ValueError("Vertices have not been computed. Call compute_diagrid first.")
        
        groups = {}
        max_roof_indices = (self.div_x + 1) * (self.div_y + 1)

        for idx in range(max_roof_indices):
            if self.vertices[idx] is None:
                continue

            pt = self.vertices[idx]
            for i, polygon in enumerate(polygons):
                if not polygon.is_planar:
                    raise ValueError(f"Polygon {i} is not planar. Please ensure all polygons are planar before deforming the roof.")
                
                if is_point_in_polygon_xy(pt, polygon):
                    groups.setdefault(i, []).append(idx)
                    break
        
        pts = []
        for polygon_idx, indices in groups.items():
            polygon = polygons[polygon_idx]
            plane = Plane(polygon[0], polygon.normal)
            for idx in indices:
                pt = self.vertices[idx]
                inter = intersection_line_plane(Line(pt, Vector(0, 0, -1)), plane)
                pts.append(Point(inter[0], inter[1], inter[2]))
                # if inter is not None:
                #     self.vertices[idx] = Point(inter[0], inter[1], inter[2])
                
        return groups, pts


    @property
    def facade_points(self):
        if not self.vertices:
            raise ValueError("Vertices have not been computed. Call compute_diagrid first.")
        
        max_indices = (self.div_x + 1) * (self.div_y + 1)
        facade_indices = list(range(0, max_indices, self.div_y + 1)) + list(range(self.div_y, max_indices, self.div_y + 1))

        return [self.vertices[idx] for idx in facade_indices]
    
    @property
    def roof_points(self):
        if not self.vertices:
            raise ValueError("Vertices have not been computed. Call compute_diagrid first.")
        
        max_indices = (self.div_x + 1) * (self.div_y + 1)
        return self.vertices[:max_indices]
        
    
    
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



class BeamCategory:
    @property
    def main_out(self):
        return "main_out"
    
    @property
    def main_in(self):
        return "main_in"
    
    @property
    def secondary_out(self):
        return "secondary_out"
    
    @property
    def secondary_in(self):
        return "secondary_in"
    

class Beam:
    def __init__(self, start_idx, end_idx, vertex_list):
        """
        Beam defined by start and end vertex indices.

        Args:
            start_idx (int): The index of the start vertex in the vertex list.
            end_idx (int): The index of the end vertex in the vertex list.
            vertex_list (VertexList)(list of Points): Entire list of vertices in space.
        """
        
        self.start_idx = start_idx
        self.end_idx = end_idx
        self.v_list = vertex_list
        
        self.frame = None
        self.category = None
        self.hierarchy = None

        self.width = None
        self.height = None
    
    @property
    def start(self):
        return self.v_list[self.start_idx]
    
    @property
    def end(self):
        return self.v_list[self.end_idx]
    
    @property
    def mid(self):
        return self.start + (self.end - self.start) / 2
    
    @property
    def axis(self):
        return Line(self.start, self.end)
    
    @property
    def direction(self):
        return self.axis.direction
    
    @property
    def length(self):
        return self.axis.length

