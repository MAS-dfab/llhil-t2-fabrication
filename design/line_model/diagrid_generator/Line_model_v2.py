from compas.geometry import Line, Vector, Box, Point

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

def consecutive_values(lst):
    new_lst = []
    for i in range(len(lst) - 1):
        new_lst.append(lst[i+1] - lst[i])
    return new_lst


##########-----------Class--------###########
class VertexList:
    def __init__(self):
        """
        A list of points representing the nodes.
        
        Args:
            vertices (list of Point)
            pairs (list of tuple): (start_idx, end_idx) representing the edges between the vertices.
        """
        self.vertices = []
        self.pairs = []


    def add(self, vertex):
        if not isinstance(vertex, Point):
            raise ValueError("Only compas Point instances can be added to VertexList.")
        self.vertices.append(vertex)
    
    def __getitem__(self, idx):
        return self.vertices[idx]

    def __len__(self):
        return len(self.vertices)
    
    def __iter__(self):
        return iter(self.vertices)

    def compute_diagrid(self, boundary, division_x=4, division_y=6, height=8.6, height_list=[]):
        if height_list and len(height_list) != division_x + 1:
            raise ValueError("Length of height_list must be equal to (division_x + 1).")
        if self.vertices:
            raise ValueError("Vertices already exist in the graph. Clear the graph before computing a new diagrid.")

        # 1. Populate vertices of the diagrid
        vec_x = boundary[1] - boundary[0]
        vec_y = boundary[-1] - boundary[0]
        vec_z = vec_x.cross(vec_y)
        vec_x.unitize()
        vec_y.unitize()
        vec_z.unitize()

        dist_x = boundary[0].distance_to_point(boundary[1])
        dist_y = boundary[0].distance_to_point(boundary[-1])
        step_x = dist_x / division_x
        step_y = dist_y / division_y
        if not height_list:
            height_list = [height / division_x] * (division_x + 1)
        else:
            height_list = consecutive_values(height_list) + [0]

        start = boundary[0]
        div_x = division_x + 1
        div_y = division_y + 1
        div_z = division_x + 1
        for i in range(div_z):  # how many levels of points in z direction
            step_z = height_list[i]
            for j in range(div_x):
                for k in range(div_y):
                    p = start + (vec_x * j * step_x) + (vec_y * k * step_y)
                    self.vertices.append(p)

            start += vec_x * step_x / 2
            start += vec_y * step_y / 2
            start -= vec_z * step_z

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

        return self.vertices, self.pairs
    
    def remove_pair(self, idx):
        for pair in self.pairs:
            if idx in pair:
                del pair
                
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
    def axis(self):
        return Line(self.start, self.end)
    
    @property
    def direction(self):
        return self.axis.direction
    
    @property
    def length(self):
        return self.axis.length
