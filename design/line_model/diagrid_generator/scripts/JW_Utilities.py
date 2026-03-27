from compas.geometry import Line


class Pair:
    def __init__(self, start_idx, end_idx, hierarchy=None, categories=[None]):
        self.start_idx = start_idx
        self.end_idx = end_idx
        
        self.hierarchy = hierarchy
        self.categories = categories

    def __getitem__(self):
        return (self.start_idx, self.end_idx)
    
    def __setitem__(self, value):
        self.start_idx, self.end_idx = value

# pair = Pair(0, 1, hierarchy=BeamHierarchy.main, categories=[BeamCategory.single, BeamCategory.CLT_attached])

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

    CLT_attached = "CLT_attached"
    single = "single"
    double = "double"
    edge = "edge"
    bracing = "bracing"
    join = "join"

######################################################################
class BeamHierarchy:
    """
    Hierarchy of beams corrsponding to the cross section dimensions.
    
    Args:
        primary (str): Primary beams that carry the main loads.
        secondary (str): Secondary beams that support the primary beams.
        tertiary (str): Tertiary beams that support the secondary beams.
    """

    main = [12, 24]


# print(BeamHierarchy.main)
# beam.width = BeamHierarchy.main[0]



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
        self.categories = pair.categories
        self.hierarchy = pair.hierarchy

        self.width = 120
        self.height = 240
        
        self.force = None
        
        self.name = None

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