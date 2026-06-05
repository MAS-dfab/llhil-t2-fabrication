"""Single screw instance for screw planning and evaluation."""
from compas.geometry import Line, Cylinder
from enum import Enum
import math

class RejectReason(Enum):
    """
    Enumeration of screw rejection reasons for logging and fallback strategies.
    
    Attributes
    ----------
    """
    
    # Hard rejections
    ENTRY_MATERIAL_TOO_THIN = 0
    EXIT_MATERIAL_TOO_THIN = 1
    SPEC_MAX_INSUFFICIENT = 2

    # Soft rejections
    EXIT_PROTRUSION = 3
    ENTRY_MATERIAL_TOO_THICK = 4



class Screw:
    def __init__(self, entry, direction, diameter):
        self.entry = entry
        self.direction = direction.unitized()
        self.diameter = diameter
        
        self.available_depth = None
        self.dist_in_entry = None
        self.dist_in_exit = None

        self.length = None

        self.joint_guid = None
        self.joint_type = None
        self.position = (None, None)

        self.is_valid = False
        self.status = "UNMODIFIED"
        self.reject_reason = None
        self.fallback_type = None

        self.is_collided = False

    @property
    def exit(self):
        if self.is_valid and self.length:
            return self.entry + self.direction * self.length
        return None
    
    @property
    def line(self):
        if self.is_valid and self.length:
            return Line(self.entry, self.exit)
        return None

    @property
    def radius(self):
        return self.diameter / 2.0
    
    @property
    def cylinder(self):
        if self.is_valid and self.length:
            return Cylinder.from_line_and_radius(self.line, self.radius)
        return None
    
    def check_distance(self, tol=1e-3):
        return math.isclose(self.length, self.dist_in_entry + self.dist_in_exit, rel_tol=tol, abs_tol=tol)
    
    def protrude(self):
        if self.reject_reason != RejectReason.EXIT_PROTRUSION:
            raise ValueError("Screw cannot be protruded as it was not rejected for exit protrusion.")
        self.is_valid = True
        self.status = "PROTRUDED"
        return self.line
    
    def counterbore(self, penetration):
        if self.reject_reason != RejectReason.ENTRY_MATERIAL_TOO_THICK:
            raise ValueError("Screw cannot be counterbored as it was not rejected for entry material thickness.")
    
        offset = penetration - self.dist_in_exit
        if offset < 0:
            raise ValueError("Penetration depth is insufficient for counterboring.")
        original_entry = self.entry
        self.entry += self.direction * offset
        self.dist_in_entry += offset
        self.dist_in_exit += offset
        
        self.is_valid = True
        self.status = "COUNTERBORED"
        return Line(original_entry, self.entry)