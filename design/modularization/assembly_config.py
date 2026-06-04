"""
Shared configuration constants for assembly model modules.
"""
from compas.tolerance import Tolerance
import System.Drawing as sd # type: ignore


# Tolerance for geometric operations in the assembly model
ASSEMBLY_MODEL_TOL = Tolerance(unit="M", absolute=0.001)

# Colors for visualization
alpha = 180

COLORS = {  
    "human":        sd.Color.FromArgb(alpha, 220, 50, 50),
    "robot":        sd.Color.FromArgb(alpha, 80, 200, 100),
    "plate":        sd.Color.FromArgb(alpha, 100, 100, 100)
}

# Priority ranks used to determine assembly logic sequencing
HIERARCHY_RANK = {
    "shoe": 0, 
    "tertiary": 2, 
    "secondary": 1, 
    "primary": 3, 
    "main_primary": 4
}

DIRECTION_RANK = {
    "A": 0, 
    "B": 1
}


