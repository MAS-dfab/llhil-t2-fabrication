"""
Shared configuration constants for timber model modules.
"""
from compas.tolerance import Tolerance


# Timber model unit and tolerance
TIMBER_MODEL_TOL = Tolerance(unit="M", absolute=0.001)

# Plate parameters
PLATE_THICKNESS = 0.10
PLATE_Z_OFFSET = 0.04

# Global parameter for finding joint candidates
MAX_JOINT_DIST = 0.055

# T-MultiStep specific
HEEL_THRESHOLD = 50

# K-Birdsmouth specific
KBIRD_MILL_DEPTH = 0.01