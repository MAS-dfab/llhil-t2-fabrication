from platform import node

from compas.datastructures import Graph
from compas.geometry import Point, Line, Vector


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
        - 3-tuple (x, u, v): Creates midpoint between u and v, connects x to it,
          reconnects children, and removes u/v (unless they are supports).
        
        New nodes created via this method get mobility='yz_free'.
        
        Parameters
        ----------
        pair : tuple
            Either (u, v) for direct edge, or (x, u, v) for midpoint creation.
        
        Returns
        -------
        int or None
            New node key if created, None otherwise.
        """
        if len(pair) == 2:
            u, v = pair
            if self.has_node(u) and self.has_node(v):
                self.add_graph_edge(u, v)
            return None

        x, u, v = pair

        if not (self.has_node(u) and self.has_node(v)):
            return None

        children_u = self.node_attribute(u, "children") or []
        children_v = self.node_attribute(v, "children") or []

        new_node = self.add_point_node_between(u, v, split_edge=False, mobility="yz_free")

        if children_u or children_v:
            candidates = set(children_u + children_v) - {x, u, v, new_node}

            # Don't delete support nodes
            if not self.node_attribute(u, "reached"):
                self.delete_node(u)
            if not self.node_attribute(v, "reached"):
                self.delete_node(v)

            self.add_graph_edge(x, new_node)

            for n in candidates:
                if self.has_node(n):
                    self.add_graph_edge(n, new_node)
        else:
            self.add_graph_edge(x, new_node)

        return new_node

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
        return list(self.nodes_where({"reached": True}))

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