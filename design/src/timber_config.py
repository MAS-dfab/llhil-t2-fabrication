"""
Shared configuration constants for timber model modules.
"""
from compas.tolerance import Tolerance


# Timber model unit and tolerance
TIMBER_MODEL_TOL = Tolerance(unit="M", absolute=0.001)

# Plate parameters
PLATE_THICKNESS = 0.10
PLATE_Z_OFFSET = 0.02

# Global parameter for finding joint candidates
MAX_JOINT_DIST = 0.065  # Based on the maximum shifted distance in the line model

# T-MultiStep specific
TMULTI_HEEL_THRESHOLD = 50
TMULTI_STEP_DEPTH = 0.015
TMULTI_RISER_ANGLE = 90

# K-Birdsmouth specific
KBIRD_MILL_DEPTH = 0.015
KBIRD_MITER_TYPE = "AVERAGE"  # 'AVERAGE', 'VERTICAL', None

# TButtJoint specific
TBUTT_ANGLE_THRESHOLD = 50