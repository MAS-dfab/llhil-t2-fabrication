"""Timber footing planning."""

from compas.geometry import (
    Frame, Line, Cylinder,
    cross_vectors, angle_vectors
)

from compas_timber.fabrication import Drilling, Slot

from timber_config import (
    FP_THICKNESS, FP_SCREW_ROW_COUNT, FP_SCREW_COLUMN_COUNT, 
    FP_SCREW_MINIMUM_SPACING, FP_SCREW_MINIMUM_OFFSET
)

import math

# -----------------------------------
# Screw Solver
# -----------------------------------
class PlateSolver:
    def __init__(self, model):
        self.model = model
        self.joints = model.joints

    def determine_screw_layout(self,
        fp_screw_row_count=None,
        fp_screw_column_count=None,
        fp_screw_minimum_spacing=None,
        fp_screw_minimum_offset=None        
        ):

        #default config
        if fp_screw_row_count is None:
            fp_screw_row_count=FP_SCREW_ROW_COUNT
        if fp_screw_column_count is None:
            fp_screw_column_count=FP_SCREW_COLUMN_COUNT
        if fp_screw_minimum_spacing is None:
            fp_screw_minimum_spacing=FP_SCREW_MINIMUM_SPACING
        if fp_screw_minimum_offset is None:
            fp_screw_minimum_offset=FP_SCREW_MINIMUM_OFFSET

        pass