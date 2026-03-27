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

class ConnectionPoint:
    def __init__(self, point: Point, parameter: float, vector: Vector):
        self.point = point
        self.parameter = parameter
        self.vector = vector

class RodSegment:
    def __init__(self, start: Point, end: Point):
        self.start = start
        self.end = end
    
    @property
    def line(self):
        return Line(self.start, self.end)

class BoundaryPolygon:
    def __init__(self, pivot: Point, points: list[Point], extension: float = 0.0):
        self.points = points
        self.pivot = pivot
        self.extension = extension
        self.frame = self.generate_polygon_frame()
        self.polygon_points = self.generate_polygon_points()
        self.lines = [Line(self.polygon_points[i], self.polygon_points[(i + 1) % len(self.polygon_points)]) for i in range(len(self.polygon_points))]

    def generate_polygon_frame(self):
        line_a = Line(self.pivot, self.points[0])
        line_b = Line(self.pivot, self.points[1])
        normal = line_a.vector.cross(line_b.vector).unitized()
        return Frame(self.pivot, normal)

    def generate_polygon_points(self):
        polygon_edge_vectors = []
        intersections = []
        
        for pt in self.points:
            vector = Vector.from_start_end(self.pivot, pt)
            reference_frame = self.frame.copy()
            reference_frame.point = pt
            normal = vector.cross(reference_frame.xaxis).unitized()
            T = Translation.from_vector(normal * self.extension)
            T_2 = Translation.from_vector(normal * -self.extension / 2)
            transformed_pt = pt.copy()
            transformed_pt.transform(T)
            line = Line(pt, transformed_pt)
            line.transform(T_2)
            polygon_edge_vectors.append(line)
        
        for i, line in enumerate(polygon_edge_vectors):
            next_line = polygon_edge_vectors[(i + 1) % len(polygon_edge_vectors)]
            intersection = intersection_line_line((line.start, line.end), (next_line.start, next_line.end))

            if not intersection or intersection[0] is None:
                print(f"No intersection found between Line {i} and Line {(i + 1) % len(polygon_edge_vectors)}.")
                continue

            intersection_point = Point(*intersection[0])
            intersections.append(intersection_point)
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
    def __init__(self, shape: Polygon, start_eccentricity: float, end_eccentricity: float, overlap:float, flipped: bool = False):
        self.shape = shape
        self.pivot = shape.pivot_point
        self.edge_points = shape.edge_points
        self.start_eccentricity = start_eccentricity
        self.end_eccentricity = end_eccentricity
        self.overlap = overlap
        self.flipped = flipped
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
    
    # @property
    # def vertices(self):
    #     return self.shape.vertices
        
    def generate_segments(self):
        self.segments = []
        self.connection_points = [] #Reset connection points

        for edge in self.edges:
            #check for pivot
            if self.pivot is not None: centroid = self.pivot
            else: centroid = self.centroid

            #find corresponding edge point
            start_point = self.edge_points[self.edges.index(edge)]

            #Calculate eccentricities
            if self.flipped == True:
                start = centroid - edge.vector * self.start_eccentricity
                end = start_point - edge.vector * self.end_eccentricity
            elif self.flipped == False:
                start = centroid + edge.vector * self.start_eccentricity
                end = start_point + edge.vector * self.end_eccentricity

            #Extend line 'inwards'
            start = start - Vector.from_start_end(start, end)

            #Create RodSegment instance, append to list
            rod_segment = RodSegment(start, end)
            self.segments.append(rod_segment)

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

            #Find intersection point of the two segments
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

        #Replace old rods with adjusted rods
        self.segments = adjusted_segments