# Line Model

Modular Python library for generating and classifying hierarchical tree structures in Grasshopper/Rhino.

## Structure

```
line_model/
├── config.py          # Shared constants and defaults
├── nodegraph.py       # NodeGraph class (extends COMPAS Graph)
├── tree_builder.py    # Tree structure generation
├── edge_classifier.py # Primary/secondary edge classification
└── data/              # Output data (JSON exports)
```

## Node Mobility System

Nodes are categorized by their design freedom for shaping workflows:

| Mobility | Description | Assigned When |
|----------|-------------|---------------|
| `fixed` | Position locked | Supports (`reached=True`) + inset corners |
| `z_free` | Can move in Z only | Apex nodes not at supports |
| `yz_free` | Move in Y and Z axis | Nodes added via `add_segment()` |

```python
# Get nodes by mobility
fixed_pts = ng.points_by_mobility("fixed")
z_free_pts = ng.points_by_mobility("z_free")
yz_free_pts = ng.points_by_mobility("yz_free")
```

## Modules

### `config.py`
Centralized configuration constants for all modules.

| Constant | Default | Description |
|----------|---------|-------------|
| `DEFAULT_DIV_X` | 8 | Grid divisions in X |
| `DEFAULT_DIV_Y` | 6 | Grid divisions in Y |
| `DEFAULT_NUM_LEVELS` | 4 | Hierarchical levels |
| `DEFAULT_INSET_EDGE` | 0.5 | Boundary corner inset |
| `DEFAULT_INSET_INTERIOR` | 0.4 | Interior corner inset |
| `DEFAULT_REACH_TOL` | 0.05 | Support snapping tolerance |
| `DEFAULT_PARALLEL_TOL` | 0.9 | Dot product threshold for primary edges |
| `DEFAULT_NEAR_THRESHOLD` | 1.0 | Distance for "near support" regions |

---

### `nodegraph.py`
Core graph data structure extending COMPAS `Graph` with spatial point indexing.

**Key Features:**
- Point deduplication via `_point_index` dictionary
- Support node tracking via `reached` attribute
- Edge grouping and line extraction

**Main Methods:**
```python
ng = NodeGraph()

# Add/get nodes with automatic deduplication
key = ng.get_or_add_point_node(Point(x, y, z), group=0, level=1)

# Add edges with attributes
ng.add_graph_edge(u, v, etype="primary", group=0)

# Get support points (nodes with reached=True)
supports = ng.get_support_points()

# Extract lines by group
lines_group_0 = ng.edge_lines_by_group(0)

# Get nodes by mobility type
fixed = ng.points_by_mobility("fixed")
z_free = ng.points_by_mobility("z_free")
xyz_free = ng.points_by_mobility("xyz_free")
```

---

### `tree_builder.py`
Generates hierarchical tree graphs from boundary, supports, and optional roof surface.

**Usage in Grasshopper:**
```python
from tree_builder import build_tree_graph

ng = build_tree_graph(
    boundary=boundary,      # Rhino curve
    supports=supports,      # List of 4 support points
    roof_brep=roof_brep,    # Optional: surface for projection
    div_x=8,                # Optional: override defaults
    div_y=6,
    num_levels=4
)

# Outputs
group_0_lines = ng.edge_lines_by_group(0)
group_1_lines = ng.edge_lines_by_group(1)
all_points = ng.node_points()
```

**How it works:**
1. Divides boundary into grid corners
2. For each level, insets corners toward apex points
3. Projects base level onto roof brep (if provided)
4. Creates edges between consecutive levels
5. Snaps apex points to nearest supports (marks as `reached=True`)

---

### `edge_classifier.py`
Classifies graph edges as primary (toward supports) or secondary (perpendicular).

**Usage in Grasshopper:**
```python
from edge_classifier import classify_single_segment

result = classify_single_segment(
    graph=ng,
    segment_index=0,
    seg_x=8,
    seg_y=2
)

primary_lines = result["primary_lines"]
secondary_lines = result["secondary_lines"]
near_support = result["near_support"]
```

**Classification Logic:**
1. **Initial pass:** Compare edge direction to direction toward nearest support
2. **Subgraph windowing:** Divide graph into spatial segments
3. **Dominant direction:** In regions far from supports, align to dominant primary direction

---

## Dependencies

- [COMPAS](https://compas.dev/) - Computational framework
- [compas_rhino](https://compas.dev/compas/latest/api/compas_rhino.html) - Rhino integration
- Rhino.Geometry (Grasshopper environment)

## Quick Start

```python
# In Grasshopper Python component
import sys
sys.path.append(r"path\to\line_model")

from tree_builder import build_tree_graph

# Build tree
ng = build_tree_graph(boundary, supports, roof_brep)

# Outputs by group
a = ng.edge_lines_by_group(0)
b = ng.edge_lines_by_group(1)

# Preview by mobility
c = ng.points_by_mobility("fixed")     # Red - locked
d = ng.points_by_mobility("z_free")    # Blue - Z only
e = ng.points_by_mobility("xyz_free")  # Green - full freedom
```

## Module Reload (Development)

When editing modules, reload to see changes:
```python
import importlib
import nodegraph, tree_builder, edge_classifier

importlib.reload(nodegraph)
importlib.reload(tree_builder)
importlib.reload(edge_classifier)

from tree_builder import build_tree_graph
# ... use updated code
```