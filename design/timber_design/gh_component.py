import os
import sys

_gh_path = ghenv.Component.OnPingDocument().FilePath
_gh_dir = os.path.dirname(_gh_path)
_this_dir = os.path.join(_gh_dir, "reciprocal_from_mesh")

if _this_dir not in sys.path:
    sys.path.insert(0, _this_dir)

import importlib
import core_graph
importlib.reload(core_graph)

from core_graph import reciprocal_from_mesh, ReciprocalFrame


# ============ GRASSHOPPER MAIN ============

if 'mesh' in dir() and mesh:
    wt, we, wf, ws = 1.0, 0.2, 0.5, 0.7

    rf = ReciprocalFrame.from_mesh(mesh, xi)  # Build graph only
    solver_info = rf.solve(weights=(wt, we, wf, ws), srf=srf)  # Then solve

    lines = rf.to_lines()
    
    print(solver_info)