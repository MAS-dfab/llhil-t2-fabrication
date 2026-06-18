"""
Shared configuration constants for timber model modules.
"""
from compas.tolerance import Tolerance


# Timber model unit and tolerance
TIMBER_MODEL_TOL = Tolerance(unit="M", absolute=0.001)

# CLT plate parameters
PLATE_THICKNESS = 0.1
PLATE_Z_OFFSET = 0.02

# Footing plate parameters
FP_THICKNESS = 0.01
FP_SCREW_ROW_COUNT = 4
FP_SCREW_COLUMN_COUNT = 2
FP_SCREW_MINIMUM_SPACING = 0.02
FP_SCREW_MINIMUM_OFFSET = 0.01

# Mid node plate parameters
MP_THICKNESS = 0.00
MP_SCREW_ROW_COUNT = 2
MP_SCREW_COLUMN_COUNT = 2
MP_SCREW_MINIMUM_SPACING = 0.02
MP_SCREW_MINIMUM_OFFSET = 0.01

# Global parameter for finding joint candidates
MAX_JOINT_DIST = 0.065  # Based on the maximum shifted distance in the line model

# T-MultiStep specific
TMULTI_HEEL_THRESHOLD = 50
TMULTI_STEP_DEPTH = 0.015
TMULTI_RISER_ANGLE = 90

# K-Birdsmouth specific
KBIRD_MILL_DEPTH = 0.01
KBIRD_MITER_TYPE = "AVERAGE"  # 'AVERAGE', 'VERTICAL', None

# T-Birdsmouth specific
TBIRD_MILL_DEPTH = 0.01

# TButtJoint specific
TBUTT_MILL_DEPTH = 0.01
TBUTT_ANGLE_THRESHOLD = 50
