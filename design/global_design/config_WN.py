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

# Extension defaults (NEW)
DEFAULT_EXTEND_WIDTH = 4.81    # 每侧紫色区域的宽度（米）
DEFAULT_EXTEND_DIV = 2        # 每侧紫色区域分几个 cell

# Edge classification defaults
DEFAULT_PARALLEL_TOL = 0.9
DEFAULT_NEAR_THRESHOLD = 1.0
DEFAULT_OVERLAP = 0.1
DEFAULT_SEG_X = 8
DEFAULT_SEG_Y = 2
DEFAULT_ANGLE_TOL = 0.15  # radians for dominant direction binning
