"""
Shared configuration constants for line model modules.
"""

# Tree builder defaults
DEFAULT_DIV_X = 8
DEFAULT_DIV_Y = 6
DEFAULT_NUM_LEVELS = 4
DEFAULT_INSET_EDGE = 0.5
DEFAULT_INSET_INTERIOR = 0.4
DEFAULT_REACH_TOL = 0.05

# Edge classification defaults
DEFAULT_PARALLEL_TOL = 0.999
DEFAULT_NEAR_THRESHOLD = 1.0
DEFAULT_OVERLAP = 0.001
DEFAULT_SEG_X = 8
DEFAULT_SEG_Y = 2
DEFAULT_ANGLE_TOL = 0.15  # radians for dominant direction binning

# Field-driven node movement defaults
DEFAULT_REPULSION_STRENGTH = 1.0  # max displacement at distance=0
DEFAULT_MAX_DISTANCE = 10.0       # nodes beyond this are unaffected
DEFAULT_MOVEMENT_ITERATIONS = 1   # number of relaxation iterations

# Per-axis strength factors (multiplied with base strength)
AXIS_FACTORS = {
    "x": 0.0,   # X-axis movement strength
    "y": 1.0,   # Y-axis movement strength
    "z": 0.45,   # Z-axis movement strength
}
