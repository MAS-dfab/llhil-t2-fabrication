import math

from compas.geometry import Line
from compas.geometry import Point
from compas.geometry import Polygon
from compas.geometry import Vector
from compas.geometry import Frame
from compas.geometry import Transformation
from compas.geometry import Translation
from compas.geometry import intersection_line_line

from compas.tolerance import TOL
from compas.datastructures import Graph

def arctangent2(vector: Vector):
    return math.atan2(vector.y, vector.x)

class ConnectionPoint:
    def __init__(self, point: Point, parameter: float, vector: Vector):
        self.point = point
        self.parameter = parameter
        self.vector = vector

class RodSegment:
    def __init__(self, start: Point, end: Point):
        self.start = start
        self.end = end
        self.midpoint = Point((start.x + end.x) / 2, (start.y + end.y) / 2, (start.z + end.z) / 2)
    
    @property
    def line(self):
        return Line(self.start, self.end)

class Rod:
    def __init__(self, segment1: RodSegment, segment2: RodSegment = None):
        self.key = None
        self.segment1 = segment1
        self.segment2 = segment2
        self.beam = None

    @property
    def line(self) -> Line:
        if self.segment2 is not None:
            return Line(self.segment1.start, self.segment2.start)
        else:
            return Line(self.segment1.start, self.segment1.end)

class BoundaryPolygon:
    def __init__(self, pivot: Point, points: list[Point], extension: float = 0.0):
        self.pivot = pivot
        self.points = self.sort_points_clockwise(points)
        self.extension = extension
        self.frame = self.generate_polygon_frame()
        self.polygon_edges = self.generate_polygon_edges()
        self.polygon_points = self.generate_polygon_points()
        self.lines = [Line(self.polygon_points[i], self.polygon_points[(i + 1) % len(self.polygon_points)]) for i in range(len(self.polygon_points))]
    
    def sort_points_clockwise(self, points: list[Point]):
        sorted_points = sorted(points, key=lambda pt: arctangent2(Vector.from_start_end(self.pivot, pt)))
        return sorted_points

    def generate_polygon_frame(self):
        line_a = Line(self.pivot, self.points[0])
        line_b = Line(self.pivot, self.points[1])
        normal = line_a.vector.cross(line_b.vector).unitized()
        return Frame(self.pivot, normal)

    def generate_polygon_edges(self):
        polygon_edges = []

        for pt in self.points:
            vector = Vector.from_start_end(self.pivot, pt)
            reference_frame = self.frame.copy()
            reference_frame.point = pt
            normal = vector.cross(reference_frame.xaxis).unitized()
            T = Translation.from_vector(normal * self.extension)
            T_2 = Translation.from_vector(normal * -self.extension)
            transformed_pt_1 = pt.copy()
            transformed_pt_1.transform(T)
            transformed_pt_2 = pt.copy()
            transformed_pt_2.transform(T_2)
            line_t = Line(transformed_pt_1, transformed_pt_2)
            polygon_edges.append(line_t)

        return polygon_edges

    def generate_polygon_points(self):
        intersections = []
        def find_intersection(line1, line2):
            intersection = intersection_line_line((line1.start, line1.end), (line2.start, line2.end))
            if not intersection or intersection[0] is None:
                print(f"No intersection found between Line {line1} and Line {line2}.")
                return None
            return Point(*intersection[0])
                
        # def sort_edges_clockwise(edges):
        #     center = self.pivot
        #     def angle_from_center(edge):
        #         mid_point = edge.midpoint
        #         vector = Vector.from_start_end(center, mid_point)
        #         return math.atan2(vector.y, vector.x)
        #     return sorted(edges, key=angle_from_center)

        def sort_edges_clockwise(edges):
            center = self.pivot
                # Sort edges based on their angle from the center
            return sorted(edges, key=lambda edge: math.atan2(Vector.dot(Vector.from_start_end(center, edge.midpoint), self.frame.zaxis), Vector.dot(Vector.from_start_end(center, edge.midpoint), self.frame.yaxis)))

        sorted_polygon_edges = sort_edges_clockwise(self.polygon_edges)

        for i, line in enumerate(sorted_polygon_edges):
            next_line = sorted_polygon_edges[(i + 1) % len(sorted_polygon_edges)]
            intersection = find_intersection(line, next_line)
            intersections.append(intersection)
        return intersections

    @property
    def centroid(self):
        x = sum(pt.x for pt in self.polygon_points) / len(self.polygon_points)
        y = sum(pt.y for pt in self.polygon_points) / len(self.polygon_points)
        z = sum(pt.z for pt in self.polygon_points) / len(self.polygon_points)
        return Point(x, y, z)

    @property
    def shape(self):
        return Polygon(self.polygon_points)
    
    @property
    def pivot_point(self):
        return self.pivot
    
    @property
    def edge_points(self):
        return self.points
    
class RFUnit:
    def __init__(self, shape: Polygon, start_eccentricity: float, end_eccentricity: float, overlap:float):
        self.shape = shape
        self.pivot = shape.pivot_point
        self.frame = shape.frame
        self.edge_points = shape.edge_points
        self.start_eccentricity = start_eccentricity
        self.end_eccentricity = end_eccentricity
        self.overlap = overlap
        self.flipped = False if start_eccentricity < 0 else True
        self.connection_points = []
        self.segments = []
        self.key = None

    @property
    def centroid(self):
        return self.shape.centroid
    
    @property
    def edges(self):
        if self.flipped:
            return list(reversed(self.shape.lines))
        else:
            return self.shape.lines

    def sort_segments(self, edges):
        center = self.pivot
                # Sort edges based on their angle from the center
        return sorted(edges, key=lambda edge: math.atan2(Vector.dot(Vector.from_start_end(center, edge.end), self.frame.zaxis), Vector.dot(Vector.from_start_end(center, edge.end), self.frame.yaxis)))
    
    def generate_segments(self):
        self.segments = [] #reset segments
        self.connection_points = [] #Reset connection points

        for edge in self.edges:
            #check for pivot
            if self.pivot is not None: centroid = self.pivot
            else: centroid = self.centroid

            #find corresponding edge point
            start_point = self.edge_points[self.edges.index(edge)]

            #Calculate eccentricities
            if self.flipped == True:
                start = centroid - edge.vector * self.start_eccentricity / edge.length #not sure if this is right, might need to be reversed
                end = start_point - edge.vector * self.end_eccentricity / edge.length
            elif self.flipped == False:
                start = centroid + edge.vector * self.start_eccentricity / edge.length
                end = start_point + edge.vector * self.end_eccentricity / edge.length

            #Extend line 'inwards'
            start = start - Vector.from_start_end(start, end)

            #Create RodSegment instance, append to list
            rod_segment = RodSegment(start, end)
            self.segments.append(rod_segment)
            self.segments = self.sort_segments(self.segments)

            #Create connection point, append to list
            _point, param = edge.closest_point(end, return_parameter = True)
            vector = Vector.from_start_end(start,end).unitized()
            connection = ConnectionPoint(end, param, vector)
            self.connection_points.append(connection)

    def adjust_segments(self, overlap = None):
        overlap = overlap if overlap is not None else self.overlap #not None check
        adjusted_segments = []

        num_segments = len(self.segments)
        for i in range(num_segments):
            #Get segments pair
            segment = self.segments[i]

            if self.flipped == True:
                next_segment = self.segments[(i - 1) % num_segments] #Wrap wrap wrap around
            elif self.flipped == False:
                next_segment = self.segments[(i + 1) % num_segments] #Wrap wrap wrap around

            #Find intersection point of the two sorted_segments
            intersection = intersection_line_line((segment.start, segment.end), (next_segment.start, next_segment.end))

            if not intersection or intersection[0] is None:
                print(f"No intersection found between Rod {i} and Rod{(i + 1) % num_segments}.")
                adjusted_segments.append(segment) #Keep original segment if no intersection
                continue

            #First intersection point
            intersection_point = Point(*intersection[0])

            #Extend rod from intersection point with specified overlap
            segment_vector = Vector.from_start_end(segment.start, intersection_point). unitized()
            extended_start = intersection_point + segment_vector *-overlap

            #Update segment start point
            segment.start = extended_start
            
            #Store adjusted rod
            adjusted_segments.append(segment)

        #Sort adjusted segments
        self.segments = self.sort_segments(adjusted_segments)

    def get_edge_frame(self, edge_index: int) -> Frame:
        point = self.connection_points[edge_index].point
        xaxis = self.edges[edge_index].vector
        yaxis = self.connection_points[edge_index].vector
        frame = Frame(point, xaxis, yaxis)
        return frame

class RFCluster(Graph):
    def __init__(self):
        self.graph = Graph(default_node_attributes={"connected_shape_edges": []})
    #     # self.vertex_configuration = vertex_configuration

    @property
    def units(self) -> list[RFUnit]:
        return self.graph.nodes_attribute(name="unit")

    def get_next_edge_index(self, unit: RFUnit, current_vertex: Point) -> int:
        # Identify all edges that are already solved
        already_used_edges = self.graph.node_attribute(unit.key, "connected_shape_edges")

        #Find the edge of new_unit that is connected to the current vertex, and is not already used in a connection
        candidate_edges = []
        for edge_index, edge in enumerate(unit.edges):
            #if edge is not already used
            if edge_index not in already_used_edges:
                #prepare candidates based on the min distance of either of its ends to the current vertex
                d1 = edge.end.distance_to_point(current_vertex)
                d2 = edge.start.distance_to_point(current_vertex)
                min_distance = min(d1, d2)
                candidate_edge_data = (edge_index, min_distance)
                candidate_edges.append(candidate_edge_data)

        sorted_candidate_edges = sorted(candidate_edges, key=lambda x: x[1])
        if len(sorted_candidate_edges) > 0:
            edge_index, distance = sorted_candidate_edges [0]
            return edge_index
        
        return None
    
    @property
    def rods(self) -> list[Rod]:
        seen = set()
        ordered_rods = []
        for unit in self.units:
            for seg in unit.segments:
                if seg.rod and seg.rod not in seen:
                    seen.add(seg.rod)
                    ordered_rods.append(seg.rod)
        
        return ordered_rods

    def create_rods_from_single_segments(self):
        for unit in self.units:
            for seg in unit.segments:
                new_rod = Rod(segment1=seg)
                seg.rod = new_rod

    def create_rods_from_shared_segments(self, angular_tolerance: float = 5.0) -> None:
        # Track segments that have already been assigned to a rod
        processed_segments = set()
        
        # Process each unit and its segments
        for unit in self.units:
            for i in range(len(unit.segments)):
                matching_segment = None
                segment = unit.segments[i]
                connection_point = unit.connection_points[i]

                # Skip if segment already processed
                if segment in processed_segments:
                    continue
                
                # Get neighboring units from graph
                neighbors = self.graph.neighbors(unit.key)
                
                # Check each neighbor for a matching segment
                matching_segment = None
                neighbor_unit = None
                for neighbor_key in neighbors:
                    neighbor_unit = self.graph.node_attribute(neighbor_key, "unit")

                    # Check each segment in neighbor
                    for j in range(len(neighbor_unit.segments)):
                        nbr_segment = neighbor_unit.segments[j]
                        nbr_connection_point = neighbor_unit.connection_points[j]

                        # Skip if neighbor segment already processed
                        if nbr_segment in processed_segments:
                            continue
                            
                        # Check if connection points match
                        if TOL.is_allclose(connection_point.point, nbr_connection_point.point):
                            # Check collinearity
                            vec1 = segment.line.vector.unitized()
                            vec2 = nbr_segment.line.vector.unitized()
                            
                            angle = vec1.angle(vec2)
                            if angle >= angular_tolerance:
                                continue

                            # We found a matching segment
                            matching_segment = nbr_segment
                            break
                    
                    if matching_segment:
                        break

                # Create appropriate rod
                if matching_segment:
                    # Create merged rod from both segments
                    new_rod = Rod(segment, matching_segment)
                    segment.rod = new_rod
                    matching_segment.rod = new_rod
                    processed_segments.add(segment)
                    processed_segments.add(matching_segment)
                else:
                    # Create single-segment rod
                    new_rod = Rod(segment)
                    segment.rod = new_rod
                    processed_segments.add(segment)