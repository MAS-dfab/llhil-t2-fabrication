from compas.datastructures import Graph
from compas.geometry import Point, Line, Vector

# Import config - handle both package and standalone execution
try:
    from . import config
except ImportError:
    import config


class NodeGraph(Graph):
    """
    Graph data structure for tree-like spatial networks with point deduplication.
    
    Extends COMPAS Graph with:
    - Automatic point deduplication via spatial indexing
    - Node mobility attributes for design workflows (fixed, z_free, xyz_free)
    - Edge grouping for visualization
    - Support node tracking via 'reached' attribute
    
    Node Attributes
    ---------------
    point : Point
        3D position of the node.
    group : int
        Group ID for edge visualization (0 or 1).
    level : int
        Hierarchical level in the tree structure.
    mobility : str
        Design freedom: 'fixed', 'z_free', or 'xyz_free'.
    reached : bool
        True if node is a support point.
    children : list
        Child node keys for parent-child relationships.
    
    Example
    -------
    >>> ng = NodeGraph()
    >>> n1 = ng.get_or_add_point_node(Point(0, 0, 0), group=0)
    >>> n2 = ng.get_or_add_point_node(Point(1, 0, 0), group=0)
    >>> ng.add_graph_edge(n1, n2)
    >>> lines = ng.edge_lines_by_group(0)
    """
    
    def __init__(self, *args, **kwargs):
        # super(NodeGraph, self).__init__(*args, **kwargs)
        super().__init__(*args, **kwargs)
        self._point_index = {}
        self._next_key = 0

    def point_key(self, pt, prec=3):
        """
        Generate a hashable key from a point for deduplication.
        
        Parameters
        ----------
        pt : Point
            Input point.
        prec : int
            Decimal precision for rounding coordinates.
        
        Returns
        -------
        tuple
            (x, y, z) rounded to precision.
        """
        return (round(pt.x, prec), round(pt.y, prec), round(pt.z, prec))

    def get_or_add_point_node(self, pt, **attr):
        """
        Get existing node at point location or create a new one.
        
        If a node already exists at the same location (within precision),
        returns that node's key and updates its attributes. Otherwise,
        creates a new node.
        
        Parameters
        ----------
        pt : Point
            3D position for the node.
        **attr
            Node attributes (group, level, mobility, reached, etc.)
        
        Returns
        -------
        int
            Node key.
        """
        pkey = self.point_key(pt)
        if pkey in self._point_index:
            key = self._point_index[pkey]
            # update attributes on existing node
            for name, value in attr.items():
                self.node_attribute(key, name, value)
            return key

        key = self._next_key
        self._next_key += 1
        self.add_node(key, x=pt.x, y=pt.y, z=pt.z, point=pt, **attr)
        self._point_index[pkey] = key
        return key

    def add_graph_edge(self, u, v, **attr):
        """
        Add an edge between two nodes if not already present.
        
        Auto-inherits 'group' from connected nodes if not specified.
        
        Parameters
        ----------
        u, v : int
            Node keys.
        **attr
            Edge attributes (etype, group, etc.)
        """
        if u == v:
            return
        if not self.has_edge((u, v)):
            # Auto-inherit group from nodes if not specified
            if "group" not in attr:
                gu = self.node_attribute(u, "group")
                gv = self.node_attribute(v, "group")
                attr["group"] = gu if gu is not None else gv
            if "etype" not in attr:
                attr["etype"] = "parent_child"
            
            self.add_edge(u, v, **attr)

    def edge_lines_by_group(self, group_id):
        """
        Extract Line objects for all edges in a group.
        
        Parameters
        ----------
        group_id : int
            Group identifier (typically 0 or 1).
        
        Returns
        -------
        list of Line
            Edge geometry for visualization.
        """
        lines = []
        for u, v in self.edges_where({"group": group_id}):
            pu = self.node_attribute(u, "point")
            pv = self.node_attribute(v, "point")
            if pu and pv:
                lines.append(Line(pu, pv))
        return lines

    def mark_y_parallel_as_double(self, threshold=0.99):
        """
        Mark edges that are parallel to the Y axis (in XY projection) as 'double'.
        
        Parameters
        ----------
        threshold : float
            Dot product threshold with Y axis (0.99 = ~8 degrees).
        
        Returns
        -------
        list of tuple
            Edges that were marked as double.
        """
        y_axis = Vector(0.0, 1.0, 0.0)
        marked = []
        
        for u, v in self.edges():
            pu = self.node_attribute(u, "point")
            pv = self.node_attribute(v, "point")
            if not pu or not pv:
                continue
            
            # Edge direction in XY only
            edge_dir = Vector(pv.x - pu.x, pv.y - pu.y, 0.0)
            if edge_dir.length < 1e-9:
                continue
            
            edge_dir.unitize()
            if abs(edge_dir.dot(y_axis)) > threshold:
                self.edge_attribute((u, v), "main_secondary", "double")
                marked.append((u, v))
        
        return marked

    def edge_lines(self):
        """
        Get all edges as Line objects.
        
        Returns
        -------
        list of Line
            All edge geometry.
        """
        lines = []
        for u, v in self.edges():
            pu = self.node_attribute(u, "point")
            pv = self.node_attribute(v, "point")
            if pu and pv:
                lines.append(Line(pu, pv))
        return lines

    def node_points(self):
        """
        Get all node positions as Point objects.
        
        Returns
        -------
        list of Point
            All node positions (may include None for nodes without points).
        """
        return list(self.nodes_attribute("point"))

    def add_point_node_between(self, u, v, t=0.5, split_edge=True, **attr):
        """
        Create a new node at a parameter along the edge between two nodes.
        
        Parameters
        ----------
        u, v : int
            Node keys defining the edge.
        t : float
            Parameter (0-1) along edge. 0.5 = midpoint.
        split_edge : bool
            If True, delete edge (u,v) and create edges (u,w) and (w,v).
        **attr
            Attributes for the new node.
        
        Returns
        -------
        int or None
            New node key, or None if u/v don't exist.
        """
        if not self.has_node(u) or not self.has_node(v):
            return None

        pu = self.node_attribute(u, "point")
        pv = self.node_attribute(v, "point")

        # Use COMPAS Line interpolation
        p = Line(pu, pv).point_at(t)

        if "group" not in attr:
            attr["group"] = self.node_attribute(u, "group")
        if "level" not in attr:
            attr["level"] = self.node_attribute(u, "level")

        w = self.get_or_add_point_node(p, **attr)

        if split_edge and self.has_edge((u, v)):
            try:
                edge_attr = self.edge_attributes((u, v))
            except:
                edge_attr = {}

            self.delete_edge((u, v))
            self.add_graph_edge(u, w, **edge_attr)
            self.add_graph_edge(w, v, **edge_attr)

        return w

    def add_segment(self, pair):
        """
        Add an edge or create an intermediate node between two nodes.
        
        Used for tree restructuring. Handles two cases:
        - 2-tuple (u, v): Creates direct edge between existing nodes.
        - 4-tuple (x, u, v, bool): Creates midpoint between u and v, connects x to it,
          if bool is True, reconnects children, and removes u/v (unless they are supports).
        
        New nodes created via this method get mobility='yz_free'.
        
        Parameters
        ----------
        pair : tuple
            Either (u, v) for direct edge, or (x, u, v, bool) for midpoint creation.
        
        Returns
        -------
        int or None
            New node key if created, None otherwise.
        """
        if len(pair) == 2:
            u, v = pair
            if self.has_node(u) and self.has_node(v):
                self.add_graph_edge(u, v)
                self.edge_attribute((u, v), "main_secondary", "primary")
            return None

        x, u, v, flag = pair
        if not (self.has_node(x) and self.has_node(u) and self.has_node(v)):
            return None
        
        new_node = self.add_point_node_between(u, v, split_edge=False, mobility="yz_free")

        if flag is False:
            self.add_graph_edge(x, new_node)
            return None

        children_u = self.node_attribute(u, "children") or []
        children_v = self.node_attribute(v, "children") or []

        new_node = self.add_point_node_between(u, v, split_edge=False, mobility="yz_free")

        if (children_u or children_v):
            candidates = set(children_u + children_v) - {x, u, v, new_node}
            # Don't delete support nodes
            if not self.node_attribute(u, "reached"):
                self.delete_node(u)
            if not self.node_attribute(v, "reached"):
                self.delete_node(v)
            new_edge = self.add_graph_edge(x, new_node)
            self.edge_attribute((x, new_node), "main_secondary", "double")

            for n in candidates:
                if self.has_node(n):
                    self.add_graph_edge(n, new_node)


        else:
            self.add_graph_edge(x, new_node)
            self.edge_attribute((x, new_node), "main_secondary", "double")


        return new_node
    
    def add_node_along_edge(self, nodes = None, dependencies = None, t=0.3, **attr):
        """
        Add a node at a parameter along the edge between two nodes.
        
        Parameters
        ----------
        u, v : int
            Node keys defining the edge.
        t : float
            Parameter (0-1) along edge. 0.5 = midpoint.
        **attr
            Attributes for the new node.
        
        Returns
        -------
        int or None
            New node key, or None if u/v don't exist.
        """
        u, v = nodes
        new_node = self.add_point_node_between(u, v, t=t, split_edge=True, **attr)
        # assign mobility='z_free' to new node by default
        self.node_attribute(new_node, "mobility", "z_free")
        if dependencies:
            for n in dependencies:
                if self.has_node(n):
                    self.add_graph_edge(n, new_node)

    

    def add_segments(self, pairs):
        """
        Apply add_segment to multiple pairs.
        
        Parameters
        ----------
        pairs : list of tuple
            List of (u, v) or (x, u, v) tuples.
        
        Returns
        -------
        list
            New node keys (None for direct edges).
        """
        return [self.add_segment(pair) for pair in pairs]

    def get_support_nodes(self):
        """
        Get all nodes marked as supports.
        
        Returns
        -------
        list of int
            Node keys where reached=True.
        """
        # Keep the support order as the rhino input
        sup_list = []
        for node in self.nodes():
            attrs = self.node_attributes(node)
            if "reached" not in attrs or "support_id" not in attrs:
                continue
            if attrs['reached'] == True:
                sup_id = attrs["support_id"]
                sup_list.append((sup_id, node))
        sorted_sup_list = sorted(sup_list, key=lambda x: x[0])

        return [pair[1] for pair in sorted_sup_list]

    def get_support_points(self):
        """
        Get Point objects for all support nodes.
        
        Returns
        -------
        list of Point
            Positions of support nodes.
        """
        return [self.node_attribute(n, "point") for n in self.get_support_nodes()
                if self.node_attribute(n, "point") is not None]

    def nodes_by_mobility(self, mobility):
        """
        Get all nodes with a specific mobility type.
        
        Parameters
        ----------
        mobility : str
            One of 'fixed', 'z_free', or 'xyz_free'.
        
        Returns
        -------
        list of int
            Node keys matching the mobility type.
        """
        return list(self.nodes_where({"mobility": mobility}))
    
    def get_mobility_vector(self, mobility, amplitude=None):
        """
        Get the mobility vector for a specific mobility type.
        
        Parameters
        ----------
        mobility : str
            One of 'fixed', 'z_free', or 'xyz_free'.
        
        Returns
        -------
        str
            Mobility vector as a vector. 'fixed' = (0,0,0), 'z_free' = (0,0,1), 'yz_free' = (0,1,1)
        """
        if amplitude == None:
            amplitude = 1
        else:
            amplitude = float(amplitude)
        
        if mobility == "fixed":
            v = Vector(0, 0, 0) 
        elif mobility == "z_free":
            v = Vector(0, 0, 1)
        elif mobility == "yz_free":
            v = Vector(0, 1, 1)
        else:
            raise ValueError("Invalid mobility type")
        
        return v * amplitude

    def points_by_mobility(self, mobility):
        """
        Get Point objects for all nodes with a specific mobility.
        
        Parameters
        ----------
        mobility : str
            One of 'fixed', 'z_free', or 'xyz_free'.
        
        Returns
        -------
        list of Point
            Positions of matching nodes.
        """
        return [self.node_attribute(n, "point") for n in self.nodes_by_mobility(mobility)
                if self.node_attribute(n, "point") is not None]

    # --------------------------------------------------
    # Topology helpers (using COMPAS Graph methods)
    # --------------------------------------------------
    def node_valency(self, node):
        """
        Get number of edges connected to a node.
        
        Parameters
        ----------
        node : int
            Node key.
        
        Returns
        -------
        int
            Number of connected edges.
        """
        return self.degree(node)

    def leaf_nodes(self):
        """
        Get nodes with valency == 1 (endpoints).
        
        Returns
        -------
        list of int
            Node keys that are endpoints.
        """
        return [n for n in self.nodes() if self.degree(n) == 1]

    def is_leaf_edge(self, edge):
        """Verify if an edge is a leaf edge."""
        leaf_nodes = self.leaf_nodes()
        return edge[0] in leaf_nodes or edge[1] in leaf_nodes
    
    def leaf_edges(self):
        """
        Get edges which included node with valency == 1.
        
        Returns
        -------
        list of tuple
            Edge keys that are endlines.
        """
        return [e for e in self.edges() if self.is_leaf_edge(e)]

    def neighbor_points(self, node):
        """
        Get Point objects of all neighbors.
        
        Parameters
        ----------
        node : int
            Node key.
        
        Returns
        -------
        list of Point
            Neighbor positions.
        """
        return [self.node_attribute(n, "point") for n in self.neighbors(node)
                if self.node_attribute(n, "point") is not None]

    def is_edge_on_brim(self, edge):
        """Check if an edge is at the top edge of the structure."""
        if not self.is_leaf_edge(edge):
            return False
        
        for node in edge:
            if self.degree(node) != 1:
                parent = node
        
        return self.degree(parent) == 5 or self.degree(parent) == 6  # or self.degree(parent) != 8


    # --------------------------------------------------
    # Serialization (using COMPAS Graph methods)
    # --------------------------------------------------
    def save(self, filepath):
        """
        Save graph to JSON file.
        
        Parameters
        ----------
        filepath : str
            Path to output file.
        """
        import json
        with open(filepath, 'w') as f:
            json.dump(self.to_data(), f, indent=2, default=str)

    @classmethod
    def load(cls, filepath):
        """
        Load graph from JSON file.
        
        Parameters
        ----------
        filepath : str
            Path to input file.
        
        Returns
        -------
        NodeGraph
            Loaded graph instance.
        """
        import json
        with open(filepath, 'r') as f:
            data = json.load(f)
        return cls.from_data(data)

    def extract_subgraph(self, nodes):
        """
        Extract a subgraph containing only specified nodes and their edges.
        
        Parameters
        ----------
        nodes : list of int
            Node keys to include.
        
        Returns
        -------
        NodeGraph
            New graph with subset of nodes/edges.
        """
        sub = NodeGraph()
        node_set = set(nodes)
        
        # Copy nodes
        for n in nodes:
            pt = self.node_attribute(n, "point")
            if pt:
                attrs = {}
                for key in ["group", "level", "mobility", "reached", "ntype"]:
                    val = self.node_attribute(n, key)
                    if val is not None:
                        attrs[key] = val
                sub.get_or_add_point_node(pt, **attrs)
        
        # Copy edges between included nodes
        for u, v in self.edges():
            if u in node_set and v in node_set:
                attrs = {}
                for key in ["group", "etype", "main_secondary"]:
                    val = self.edge_attribute((u, v), key)
                    if val is not None:
                        attrs[key] = val
                # Re-lookup node keys in subgraph
                pu = self.node_attribute(u, "point")
                pv = self.node_attribute(v, "point")
                if pu and pv:
                    su = sub.get_or_add_point_node(pu)
                    sv = sub.get_or_add_point_node(pv)
                    sub.add_graph_edge(su, sv, **attrs)
        
        return sub

    # --------------------------------------------------
    # Field-driven node movement
    # --------------------------------------------------
    
    @staticmethod
    def constrain_vector_by_mobility(vector, mobility):
        """
        Constrain a displacement vector based on mobility type.
        
        Parameters
        ----------
        vector : Vector
            Input displacement vector.
        mobility : str
            One of 'fixed', 'z_free', 'yz_free', or 'xyz_free'.
        
        Returns
        -------
        Vector
            Constrained displacement vector.
        """
        if mobility == "fixed":
            return Vector(0, 0, 0)
        elif mobility == "z_free":
            return Vector(0, 0, vector.z)
        elif mobility == "yz_free":
            return Vector(0, vector.y, vector.z)
        elif mobility == "xyz_free":
            return Vector(vector.x, vector.y, vector.z)
        else:
            # Unknown mobility type - treat as fixed for safety
            return Vector(0, 0, 0)

    def compute_brep_distance(self, node, breps):
        """
        Compute distance and direction from a node to the nearest Brep.
        
        Parameters
        ----------
        node : int
            Node key.
        breps : list of Rhino.Geometry.Brep
            Breps to compute distance from.
        
        Returns
        -------
        tuple (float, Vector) or (None, None)
            Distance to nearest Brep and direction vector pointing FROM Brep TO node.
            Returns (None, None) if node has no point or breps is empty.
        """
        import Rhino.Geometry as rg  # type: ignore
        
        pt = self.node_attribute(node, "point")
        if pt is None or not breps:
            return None, None
        
        rg_pt = rg.Point3d(pt.x, pt.y, pt.z)
        
        min_dist = float('inf')
        closest_pt = None
        
        for brep in breps:
            if brep is None:
                continue
            # Brep.ClosestPoint returns multiple values; find the Point3d
            result = brep.ClosestPoint(rg_pt)
            
            # Handle different return formats
            cp = None
            if isinstance(result, rg.Point3d):
                cp = result
            elif isinstance(result, tuple):
                # Find the Point3d in the tuple (position varies by Rhino version)
                for item in result:
                    if isinstance(item, rg.Point3d):
                        cp = item
                        break
            
            if cp is not None:
                dist = rg_pt.DistanceTo(cp)
                if dist < min_dist:
                    min_dist = dist
                    closest_pt = cp
        
        if closest_pt is None:
            return None, None
        
        # Direction from brep to node (for repulsion, node moves in this direction)
        direction = Vector(
            rg_pt.X - closest_pt.X,
            rg_pt.Y - closest_pt.Y,
            rg_pt.Z - closest_pt.Z
        )
        
        # Normalize if length > 0
        if direction.length > 1e-9:
            direction.unitize()
        
        return min_dist, direction

    def compute_point_distance(self, node, points):
        """
        Compute distance and direction from a node to the nearest point.
        
        Parameters
        ----------
        node : int
            Node key.
        points : list of Point3d or Point
            Points to compute distance from.
        
        Returns
        -------
        tuple (float, Vector) or (None, None)
            Distance to nearest point and direction vector pointing FROM point TO node.
        """
        pt = self.node_attribute(node, "point")
        if pt is None or not points:
            return None, None
        
        min_dist = float('inf')
        closest_pt = None
        
        for p in points:
            if p is None:
                continue
            # Handle both COMPAS Point and Rhino Point3d
            px = getattr(p, 'X', None) or getattr(p, 'x', 0)
            py = getattr(p, 'Y', None) or getattr(p, 'y', 0)
            pz = getattr(p, 'Z', None) or getattr(p, 'z', 0)
            
            dist = ((pt.x - px)**2 + (pt.y - py)**2 + (pt.z - pz)**2)**0.5
            if dist < min_dist:
                min_dist = dist
                closest_pt = (px, py, pz)
        
        if closest_pt is None:
            return None, None
        
        # Direction from point to node (for repulsion)
        direction = Vector(
            pt.x - closest_pt[0],
            pt.y - closest_pt[1],
            pt.z - closest_pt[2]
        )
        
        if direction.length > 1e-9:
            direction.unitize()
        
        return min_dist, direction

    def compute_curve_distance(self, node, curves):
        """
        Compute distance and direction from a node to the nearest curve.
        
        Parameters
        ----------
        node : int
            Node key.
        curves : list of Rhino.Geometry.Curve
            Curves to compute distance from.
        
        Returns
        -------
        tuple (float, Vector) or (None, None)
            Distance to nearest curve and direction vector pointing FROM curve TO node.
        """
        import Rhino.Geometry as rg  # type: ignore
        
        pt = self.node_attribute(node, "point")
        if pt is None or not curves:
            return None, None
        
        rg_pt = rg.Point3d(pt.x, pt.y, pt.z)
        
        min_dist = float('inf')
        closest_pt = None
        
        for crv in curves:
            if crv is None:
                continue
            success, t = crv.ClosestPoint(rg_pt)
            if success:
                cp = crv.PointAt(t)
                dist = rg_pt.DistanceTo(cp)
                if dist < min_dist:
                    min_dist = dist
                    closest_pt = cp
        
        if closest_pt is None:
            return None, None
        
        direction = Vector(
            rg_pt.X - closest_pt.X,
            rg_pt.Y - closest_pt.Y,
            rg_pt.Z - closest_pt.Z
        )
        
        if direction.length > 1e-9:
            direction.unitize()
        
        return min_dist, direction

    @staticmethod
    def compute_falloff(dist, max_distance, strength, falloff="inverse_square"):
        """
        Compute repulsion magnitude based on distance with different falloff curves.
        
        Parameters
        ----------
        dist : float
            Distance from geometry.
        max_distance : float
            Maximum influence distance.
        strength : float
            Base strength multiplier.
        falloff : str
            Falloff type: 'linear', 'inverse_square', 'constant', 'smooth'
        
        Returns
        -------
        float
            Repulsion magnitude.
        """
        if dist >= max_distance:
            return 0.0
        
        # Normalized distance (0 at geometry, 1 at max_distance)
        t = dist / max_distance
        
        if falloff == "linear":
            # Linear: strength at dist=0, zero at max_distance
            return strength * (1.0 - t)
        
        elif falloff == "inverse_square":
            # Inverse square: very strong close, drops off quickly
            # Add small epsilon to avoid division by zero
            normalized = (dist + 0.1) / max_distance
            return strength * (1.0 / (normalized * normalized)) * (1.0 - t)
        
        elif falloff == "constant":
            # Constant: same strength everywhere within range
            return strength
        
        elif falloff == "smooth":
            # Smooth (cosine): gentle S-curve falloff
            import math
            return strength * (0.5 + 0.5 * math.cos(t * math.pi))
        
        else:
            # Default to linear
            return strength * (1.0 - t)

    def compute_repulsion_vector(self, node, breps, max_distance=10.0, strength=1.0, falloff="inverse_square"):
        """
        Compute repulsion vector for a node based on Brep proximity.
        
        Parameters
        ----------
        node : int
            Node key.
        breps : list of Rhino.Geometry.Brep
            Breps to repel from.
        max_distance : float
            Nodes beyond this distance are unaffected.
        strength : float
            Maximum displacement magnitude at distance=0.
        falloff : str
            Falloff type: 'linear', 'inverse_square', 'constant', 'smooth'
        
        Returns
        -------
        Vector
            Repulsion vector (unconstrained by mobility).
        """
        dist, direction = self.compute_brep_distance(node, breps)
        
        if dist is None or direction is None:
            return Vector(0, 0, 0)
        
        # Compute magnitude with selected falloff
        magnitude = self.compute_falloff(dist, max_distance, strength, falloff)
        
        # Scale direction by magnitude
        return Vector(
            direction.x * magnitude,
            direction.y * magnitude,
            direction.z * magnitude
        )

    def move_node(self, node, vector, axis_factors=None):
        """
        Move a node by a displacement vector, respecting its mobility constraint.
        
        Updates both the node's 'point' attribute and the internal point index
        for spatial deduplication consistency.
        
        Parameters
        ----------
        node : int
            Node key.
        vector : Vector
            Displacement vector (will be constrained by node's mobility).
        axis_factors : dict, optional
            Per-axis strength multipliers {"x": float, "y": float, "z": float}.
            If None, uses config.AXIS_FACTORS.
        
        Returns
        -------
        tuple (Point, Point, float) or None
            (old_point, new_point, displacement_magnitude), or None if node
            doesn't exist or has no point.
        """
        
        pt = self.node_attribute(node, "point")
        if pt is None:
            return None
        
        mobility = self.node_attribute(node, "mobility") or "fixed"
        
        # Use provided axis_factors or fall back to config
        if axis_factors is None:
            axis_factors = config.AXIS_FACTORS
        
        # Apply per-axis strength factors
        scaled_vector = Vector(
            vector.x * axis_factors.get("x", 1.0),
            vector.y * axis_factors.get("y", 1.0),
            vector.z * axis_factors.get("z", 1.0)
        )
        
        # Apply direction constraint based on mobility
        constrained = self.constrain_vector_by_mobility(scaled_vector, mobility)
        
        # If no movement, return early
        if constrained.length < 1e-9:
            return (pt, pt, 0.0)
        
        # Calculate new position
        new_pt = Point(
            pt.x + constrained.x,
            pt.y + constrained.y,
            pt.z + constrained.z
        )
        
        # Update point index: remove old key, add new key
        old_pkey = self.point_key(pt)
        new_pkey = self.point_key(new_pt)
        
        if old_pkey in self._point_index:
            del self._point_index[old_pkey]
        self._point_index[new_pkey] = node
        
        # Update node attributes
        self.node_attribute(node, "point", new_pt)
        self.node_attribute(node, "x", new_pt.x)
        self.node_attribute(node, "y", new_pt.y)
        self.node_attribute(node, "z", new_pt.z)
        
        return (pt, new_pt, constrained.length)

    def move_nodes_from_breps(self, breps, max_distance=10.0, strength=1.0, 
                               iterations=1, mobility_filter=None,
                               axis_factors=None, falloff="inverse_square"):
        """
        Move mobile nodes away from Breps based on proximity.
        
        Main API for field-driven node movement. Nodes closer to Breps
        experience stronger repulsion, constrained by their mobility attribute.
        
        Parameters
        ----------
        breps : list of Rhino.Geometry.Brep
            Repulsion geometry. Nodes move away from these surfaces.
        max_distance : float
            Nodes beyond this distance are unaffected.
        strength : float
            Maximum displacement at distance=0.
        iterations : int
            Number of movement iterations (for relaxation).
        mobility_filter : list of str, optional
            Only move nodes with these mobility types. If None, moves all
            non-fixed nodes (z_free, yz_free, xyz_free).
        axis_factors : dict, optional
            Per-axis strength multipliers {"x": float, "y": float, "z": float}.
            If None, uses config.AXIS_FACTORS.
        falloff : str
            Falloff type: 'linear', 'inverse_square', 'constant', 'smooth'
        
        Returns
        -------
        tuple (NodeGraph, dict)
            - The modified graph (self) for chaining or GH output.
            - Results dict: {node_key: {'old': Point, 'new': Point, 'displacement': float}}
              Only includes nodes that actually moved.
        
        Example
        -------
        >>> # In Grasshopper Python component:
        >>> import Rhino.Geometry as rg
        >>> # breps = list of Brep inputs
        >>> graph, results = ng.move_nodes_from_breps(
        ...     breps, max_distance=5.0, strength=0.5,
        ...     axis_factors={"x": 0.0, "y": 0.5, "z": 1.0}  # Only move in Y/Z
        ... )
        >>> lines = graph.edge_lines()
        >>> moved_points = [r['new'] for r in results.values()]
        """
        if not breps:
            return self, {}
        
        # Filter out None breps
        breps = [b for b in breps if b is not None]
        if not breps:
            return self, {}
        
        # Use provided axis_factors or fall back to config
        if axis_factors is None:
            axis_factors = config.AXIS_FACTORS
        
        # Determine which mobility types to process
        if mobility_filter is None:
            mobility_filter = ["z_free", "yz_free", "xyz_free"]
        
        # Collect nodes to process
        nodes_to_move = []
        for mob in mobility_filter:
            nodes_to_move.extend(self.nodes_by_mobility(mob))
        nodes_to_move = list(set(nodes_to_move))  # Remove duplicates
        
        results = {}
        
        for _ in range(iterations):
            for node in nodes_to_move:
                # Compute repulsion vector
                repulsion = self.compute_repulsion_vector(
                    node, breps, max_distance, strength, falloff
                )
                
                # Apply movement (mobility constraint handled inside move_node)
                result = self.move_node(node, repulsion, axis_factors=axis_factors)
                
                if result and result[2] > 1e-9:  # Actually moved
                    old_pt, new_pt, disp = result
                    results[node] = {
                        'old': old_pt,
                        'new': new_pt,
                        'displacement': disp
                    }
        
        return self, results

    def move_nodes_from_points(self, points, max_distance=10.0, strength=1.0,
                                iterations=1, mobility_filter=None, axis_factors=None,
                                falloff="inverse_square"):
        """
        Move mobile nodes away from points based on proximity.
        
        EASIEST method - just place points in Rhino where you want to repel nodes.
        
        Parameters
        ----------
        points : list of Point3d or Point
            Repulsion points. Nodes move away from these.
        max_distance : float
            Nodes beyond this distance are unaffected.
        strength : float
            Maximum displacement at distance=0.
        iterations : int
            Number of movement iterations.
        mobility_filter : list of str, optional
            Only move nodes with these mobility types.
        axis_factors : dict, optional
            Per-axis strength multipliers {"x": float, "y": float, "z": float}.
        falloff : str
            Falloff type: 'linear', 'inverse_square', 'constant', 'smooth'
        
        Returns
        -------
        tuple (NodeGraph, dict)
            Modified graph and movement results.
        
        Example
        -------
        >>> # Place points in Rhino, reference them in GH
        >>> graph, results = ng.move_nodes_from_points(pts, max_distance=5.0, strength=0.5)
        """
        if not points:
            return self, {}
        
        points = [p for p in points if p is not None]
        if not points:
            return self, {}
        
        if axis_factors is None:
            axis_factors = config.AXIS_FACTORS
        
        if mobility_filter is None:
            mobility_filter = ["z_free", "yz_free", "xyz_free"]
        
        nodes_to_move = []
        for mob in mobility_filter:
            nodes_to_move.extend(self.nodes_by_mobility(mob))
        nodes_to_move = list(set(nodes_to_move))
        
        results = {}
        
        for _ in range(iterations):
            for node in nodes_to_move:
                dist, direction = self.compute_point_distance(node, points)
                
                if dist is None or direction is None:
                    continue
                
                magnitude = self.compute_falloff(dist, max_distance, strength, falloff)
                if magnitude < 1e-9:
                    continue
                    
                repulsion = Vector(
                    direction.x * magnitude,
                    direction.y * magnitude,
                    direction.z * magnitude
                )
                
                result = self.move_node(node, repulsion, axis_factors=axis_factors)
                
                if result and result[2] > 1e-9:
                    old_pt, new_pt, disp = result
                    results[node] = {'old': old_pt, 'new': new_pt, 'displacement': disp}
        
        return self, results

    def move_nodes_from_curves(self, curves, max_distance=10.0, strength=1.0,
                                iterations=1, mobility_filter=None, axis_factors=None,
                                falloff="inverse_square"):
        """
        Move mobile nodes away from curves based on proximity.
        
        Draw curves in Rhino to define exclusion zones - nodes move away from curves.
        
        Parameters
        ----------
        curves : list of Rhino.Geometry.Curve
            Repulsion curves. Nodes move away from these.
        max_distance : float
            Nodes beyond this distance are unaffected.
        strength : float
            Maximum displacement at distance=0.
        iterations : int
            Number of movement iterations.
        mobility_filter : list of str, optional
            Only move nodes with these mobility types.
        axis_factors : dict, optional
            Per-axis strength multipliers {"x": float, "y": float, "z": float}.
        falloff : str
            Falloff type: 'linear', 'inverse_square', 'constant', 'smooth'
        
        Returns
        -------
        tuple (NodeGraph, dict)
            Modified graph and movement results.
        
        Example
        -------
        >>> # Draw a curve in Rhino where elevator/stair should go
        >>> graph, results = ng.move_nodes_from_curves(crvs, max_distance=3.0, strength=1.0)
        """
        if not curves:
            return self, {}
        
        curves = [c for c in curves if c is not None]
        if not curves:
            return self, {}
        
        if axis_factors is None:
            axis_factors = config.AXIS_FACTORS
        
        if mobility_filter is None:
            mobility_filter = ["z_free", "yz_free", "xyz_free"]
        
        nodes_to_move = []
        for mob in mobility_filter:
            nodes_to_move.extend(self.nodes_by_mobility(mob))
        nodes_to_move = list(set(nodes_to_move))
        
        results = {}
        
        for _ in range(iterations):
            for node in nodes_to_move:
                dist, direction = self.compute_curve_distance(node, curves)
                
                if dist is None or direction is None:
                    continue
                
                magnitude = self.compute_falloff(dist, max_distance, strength, falloff)
                if magnitude < 1e-9:
                    continue
                    
                repulsion = Vector(
                    direction.x * magnitude,
                    direction.y * magnitude,
                    direction.z * magnitude
                )
                
                result = self.move_node(node, repulsion, axis_factors=axis_factors)
                
                if result and result[2] > 1e-9:
                    old_pt, new_pt, disp = result
                    results[node] = {'old': old_pt, 'new': new_pt, 'displacement': disp}
        
        return self, results

    def move_nodes_from_geometry(self, geometry, max_distance=10.0, strength=1.0,
                                  iterations=1, mobility_filter=None, axis_factors=None,
                                  falloff="inverse_square"):
        """
        Move nodes away from any geometry type (auto-detects points, curves, breps).
        
        Universal method - just pass whatever geometry you have.
        
        Parameters
        ----------
        geometry : list
            Mix of Points, Curves, and/or Breps.
        max_distance : float
            Nodes beyond this distance are unaffected.
        strength : float
            Maximum displacement at distance=0.
        iterations : int
            Number of movement iterations.
        mobility_filter : list of str, optional
            Only move nodes with these mobility types.
        axis_factors : dict, optional
            Per-axis strength multipliers.
        falloff : str
            Falloff type: 'linear', 'inverse_square', 'constant', 'smooth'
        
        Returns
        -------
        tuple (NodeGraph, dict)
            Modified graph and combined movement results.
        """
        import Rhino.Geometry as rg # type: ignore
        
        if not geometry:
            return self, {}
        
        # Sort geometry by type
        points = []
        curves = []
        breps = []
        
        for geo in geometry:
            if geo is None:
                continue
            if isinstance(geo, rg.Point3d):
                points.append(geo)
            elif isinstance(geo, Point):
                points.append(geo)
            elif isinstance(geo, rg.Curve):
                curves.append(geo)
            elif isinstance(geo, rg.Brep):
                breps.append(geo)
        
        all_results = {}
        
        # Apply each geometry type
        if points:
            _, results = self.move_nodes_from_points(
                points, max_distance, strength, iterations, mobility_filter, axis_factors, falloff
            )
            all_results.update(results)
        
        if curves:
            _, results = self.move_nodes_from_curves(
                curves, max_distance, strength, iterations, mobility_filter, axis_factors, falloff
            )
            all_results.update(results)
        
        if breps:
            _, results = self.move_nodes_from_breps(
                breps, max_distance, strength, iterations, mobility_filter, axis_factors, falloff
            )
            all_results.update(results)
        
        return self, all_results

    def get_displacement_vectors(self, results):
        """
        Extract displacement vectors from move_nodes_from_breps results.
        
        Useful for visualization in Grasshopper.
        
        Parameters
        ----------
        results : dict
            Output from move_nodes_from_breps().
        
        Returns
        -------
        list of tuple (Point, Vector)
            (start_point, displacement_vector) for each moved node.
        """
        vectors = []
        for node, data in results.items():
            old_pt = data['old']
            new_pt = data['new']
            vec = Vector(
                new_pt.x - old_pt.x,
                new_pt.y - old_pt.y,
                new_pt.z - old_pt.z
            )
            vectors.append((old_pt, vec))
        return vectors