


class ScrewSpecification:
    """Engineering parameters in Meter based on Swiss LMP Connection Rules."""
    SPEC_TABLE = {
        "WT-plus-6.5": {
            "a1": 0.030, "a1_cg": 0.030, "a2_cg": 0.024, "a2_red": 0.009,
            "min_widths": {1: 0.057, 2: 0.087, 3: 0.117}  # {pair_count: min_width}
        },
        "WT-plus-8.5": {
            "a1": 0.040, "a1_cg": 0.040, "a2_cg": 0.032, "a2_red": 0.012,
            "min_widths": {1: 0.076, 2: 0.116, 3: 0.156}
        },
        "WR-9": {
            "a1": 0.045, "a1_cg": 0.045, "a2_cg": 0.027, "a2_red": 0.014,
            "min_widths": {1: 0.068, 2: 0.113, 3: 0.158}
        },
        "WR-13": {
            "a1": 0.065, "a1_cg": 0.065, "a2_cg": 0.039, "a2_red": 0.020,
            "min_widths": {1: 0.098, 2: 0.163, 3: 0.228}
        },
    }
    ANGLE_THRESHOLD = 45
    DRILLING_DIAMETER = 0.004
    SCREW_DIAMETER = 0.0065
    SCREW_LENGTHS = [0.10, 0.13, 0.16]  # not confirmed yet
    BACK_THRESHOLD = 0.015

    def __init__(self, entry_type=None, spec_model="WT-plus-6.5"):
        if spec_model not in self.SPEC_TABLE:
            raise ValueError(f"Unsupported screw model: {spec_model}")
        
        self.entry_type = entry_type
        self.spec_model = spec_model
        self.spec_table = self.SPEC_TABLE[spec_model]

        # Minimum parameters
        if entry_type == "aligned":
            self.a1 = 0.041
            self.a1_cg = 0.065
            self.a2 = 0.050
            self.a2_cg = 0.025
            self.a2_red = None

            self.side_angle = None
            self.side_offset = None
            
        elif entry_type == "crossed":
            self.a1 = self.spec_table["a1"]
            self.a1_cg = self.spec_table["a1_cg"]
            self.a2 = None
            self.a2_cg = self.spec_table["a2_cg"]
            self.a2_red = self.spec_table["a2_red"]

            self.side_angle = 30
            self.side_offset = 0.060