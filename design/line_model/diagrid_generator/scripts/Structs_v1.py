### Edited by Jerry on 27 Mar 2026
from compas.geometry import Line
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class Pair:
    start_idx: int
    end_idx: int
    cross_section: Optional[tuple] = None
    categories: List[str] = field(default_factory=list)

    def __getitem__(self, idx):
        return (self.start_idx, self.end_idx)[idx]
    
    def __iter__(self):
        yield self.start_idx
        yield self.end_idx


class BeamCategory:
    """
    Categorizes beams.
    
    Args:
        CLT_attached (str): Beams attached to CLT panels.
        single (str): Do not need to be doubled the beams.
        double (str): Where you need to double and offset the beams.
        edge (str): Beams at the facade position.
        bracing (str): Bracing beams, e.g. around the beams near to the supports.
        join (str): Beams that are treated as one continuous element for fabrication, but be segmented for structural analysis.
    """

    CLT_ATTACHED = "CLT_attached"
    SINGLE = "single"
    DOUBLE = "double"
    EDGE = "edge"
    BRACING = "bracing"
    JOIN = "join"

######################################################################
class CrossSection:
    """
    Beam cross sections in mm.
    These are the base single-beam sections.
    Double beams are formed by placing two identical sections in parallel.
    """

    XS = (80, 200)   # smallest single section
    S  = (80, 280)   # deeper but still narrow
    M  = (100, 240)  # medium section
    L  = (120, 320)  # largest single section

    ALL = [XS, S, M, L]

# print(CrossSection.main)
# beam.width = CrossSection.main[0]



class Beam:
    def __init__(self, pair, vertex_list):
        """
        Beam defined by start and end vertex indices.

        Args:
            pair (Pair): The pair object containing start and end vertex indices.
            vertex_list (VertexList)(list of Points): Entire list of vertices in space.
        """
        self.pair = pair
        self.start_idx = pair.start_idx
        self.end_idx = pair.end_idx
        self.v_list = vertex_list
        
        self.frame = None
        self.id = None
        self.categories = pair.categories
        self.cross_section = pair.cross_section

        self.width = self.cross_section[0] if self.cross_section is not None else 80
        self.height = self.cross_section[1] if self.cross_section is not None else 200
        
        self.force = None


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



class Transform:
    def __init__(self, valency, groups, vextex_list):
        self.valency = valency
        self.groups = groups
        self.v_list = vextex_list

    def five(self):
        for parent, children in self.groups.items():
            edges = [Line(self.v_list[parent], self.v_list[c]) for c in children]