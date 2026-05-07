import math

from compas.data import json_dump
from compas.geometry import Box
from compas.geometry import Frame
from compas.datastructures import Mesh
from compas.geometry import Point
from compas.geometry import Scale
from compas.geometry import Transformation
from compas.geometry import Vector
from compas.tolerance import TOL
from compas_fab.robots import RigidBody
from compas_fab.robots import RigidBodyState
from core.base_planner import BaseRobotPlanner


class TimberProcessPlanner(BaseRobotPlanner):
    def __init__(self, group="robot12_eaXYZ"):
        super().__init__(group=group)
        
        # Inject project-specific constraints into the base planner's default options
        self.default_options["path_constraints"] = self.global_constraints
        
        # Base Frames (TODO: Move to config.json later)
        self.at_frame = Frame(point=Point(x=5.96989, y=10.56825, z=0.36166), xaxis=Vector(x=1.000, y=0.000, z=0.000), yaxis=Vector(x=-0.000, y=-1.000, z=0.000))
        self.at_T = Transformation.from_frame(self.at_frame)
        
        ps1_frame = Frame(point=Point(x=16.040, y=7.076, z=0.449), xaxis=Vector(x=-1.000, y=-0.000, z=-0.000), yaxis=Vector(x=0.000, y=-1.000, z=0.000))
        ps1_frame.rotate(math.radians(180), ps1_frame.xaxis, ps1_frame.point)
        ps1_T = Transformation.from_frame(ps1_frame)
        ps2_frame = Frame(point=Point(x=10.883, y=12.499, z=0.449), xaxis=Vector(x=-1.000, y=-0.000, z=-0.000), yaxis=Vector(x=0.000, y=-1.000, z=0.000))
        ps2_frame.rotate(math.radians(180), ps2_frame.xaxis, ps2_frame.point)
        ps2_T = Transformation.from_frame(ps2_frame)
        self.ps_frames = [ps1_frame, ps2_frame]
        self.ps_Ts = [ps1_T, ps2_T]

    def setup_physical_cell(self):
        """Loads the project-specific tool and static scene geometry into the robot cell."""
        print("Setting up physical cell geometry...")
        
        # 1. Add Tool
        tool_mesh = Mesh.from_stl("fabrication\\data\\gripper\\GripperLong_viz.stl")
        col_mesh = Mesh.from_stl("fabrication\\data\\gripper\\GripperLong_col.stl")
        tool_frame = Frame([0.000, 0.000, 0.157],  [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
        self.add_tool_to_robot(
            viz_mesh=tool_mesh,
            col_mesh=col_mesh,
            tool_frame=tool_frame,
            tool_name="GripperLong",
            connected_to="robot12_tool0"
        )
        
        # 2. Add Static Scene Objects (CNC/PUS/AT geometry)
        try:
            scene_mesh = Mesh.from_stl("fabrication\\data\\col_mesh\\t2_rfl_colmesh.stl")
            scene_rb = RigidBody.from_mesh(scene_mesh)
            self.robot_cell.rigid_body_models["t2_rfl_colmesh"] = scene_rb
            self.state.rigid_body_states["t2_rfl_colmesh"] = RigidBodyState(Frame.worldXY())
            
            # Update the planner with the new scene
            self.planner.set_robot_cell(self.robot_cell)
            self.planner.set_robot_cell_state(self.state)
            print("Scene objects added.")
        except FileNotFoundError:
            print("Warning: Could not find t2_rfl_colmesh.stl. Proceeding without scene collisions.")
            
        print("Physical cell setup complete.")

    def add_rb_to_cell(self, meshes, name):
        if name in self.robot_cell.rigid_body_models:
            del self.robot_cell.rigid_body_models[name]
        if name in self.state.rigid_body_states:
            del self.state.rigid_body_states[name]
        if len(meshes) > 0:
            meshes_RB = RigidBody.from_meshes(meshes)
            self.robot_cell.rigid_body_models[name] = meshes_RB
            self.state.rigid_body_states[name] = RigidBodyState(Frame.worldXY())
            self.planner.set_robot_cell(self.robot_cell)
            self.planner.set_robot_cell_state(self.state)
            print("RigibBodies added to robot cell.")
            return meshes_RB

    # =========================================================================
    # HARDCODED CONFIGURATIONS (TODO: Extract to config file)
    # =========================================================================
    
    @property
    def safe_configuration(self):
        configuration = self.robot_cell.zero_configuration(group=self.group)
        self.robot_cell.configuration_to_full_configuration(configuration)
        configuration = self.state.robot_configuration
        configuration["bridge1_joint_EA_X"] = 10
        configuration["robot12_joint_EA_Y"] = -6
        configuration["robot12_joint_EA_Z"] = -5
        configuration["robot12_joint_1"] = math.radians(-90)
        configuration["robot12_joint_2"] = math.radians(-55)
        configuration["robot12_joint_3"] = math.radians(55)
        configuration["robot12_joint_4"] = math.radians(180)
        configuration["robot12_joint_5"] = math.radians(90)
        configuration["robot12_joint_6"] = math.radians(-90)

        configuration["bridge2_joint_EA_X"] = 30

        """ get robot_11 out of the way """
        configuration["robot11_joint_EA_Y"] = -0.5
        configuration["robot11_joint_EA_Z"] = -4.5
        configuration["robot11_joint_1"] = math.radians(0)
        configuration["robot11_joint_2"] = math.radians(90)
        configuration["robot11_joint_3"] = math.radians(0)
        configuration["robot11_joint_4"] = math.radians(180)
        configuration["robot11_joint_5"] = math.radians(0)
        configuration["robot11_joint_6"] = math.radians(-90)
        return configuration

    @property
    def cnc_configuration(self):
        cnc_full_configuration = self.current_configuration.copy()
        cnc_full_configuration["bridge1_joint_EA_X"] = 11.1
        cnc_full_configuration["robot12_joint_EA_Y"] = -3.5
        cnc_full_configuration["robot12_joint_EA_Z"] = -4.9
        cnc_full_configuration["robot12_joint_1"] = math.radians(90)
        cnc_full_configuration["robot12_joint_2"] = math.radians(-55)
        cnc_full_configuration["robot12_joint_3"] = math.radians(55)
        cnc_full_configuration["robot12_joint_4"] = math.radians(180)
        cnc_full_configuration["robot12_joint_5"] = math.radians(90)
        cnc_full_configuration["robot12_joint_6"] = math.radians(-90)
        return cnc_full_configuration
    
    @property
    def cnc_safe_configuration(self):
        cnc_safe_full_configuration = self.current_configuration.copy()
        cnc_safe_full_configuration["bridge1_joint_EA_X"] = 11.1
        cnc_safe_full_configuration["robot12_joint_EA_Y"] = -3.5
        cnc_safe_full_configuration["robot12_joint_EA_Z"] = -4.5
        cnc_safe_full_configuration["robot12_joint_1"] = math.radians(0)
        cnc_safe_full_configuration["robot12_joint_2"] = math.radians(90)
        cnc_safe_full_configuration["robot12_joint_3"] = math.radians(0)
        cnc_safe_full_configuration["robot12_joint_4"] = math.radians(180)
        cnc_safe_full_configuration["robot12_joint_5"] = math.radians(0)
        cnc_safe_full_configuration["robot12_joint_6"] = math.radians(-90)
        return cnc_safe_full_configuration
    
    @property
    def pick_configuration(self):
        pick_full_configuration = self.current_configuration.copy()
        pick_full_configuration["bridge1_joint_EA_X"] = 9.5
        pick_full_configuration["robot12_joint_EA_Y"] = -5.5
        pick_full_configuration["robot12_joint_EA_Z"] = -3.5
        pick_full_configuration["robot12_joint_1"] = math.radians(-5)
        pick_full_configuration["robot12_joint_2"] = math.radians(0)
        pick_full_configuration["robot12_joint_3"] = math.radians(-6)
        pick_full_configuration["robot12_joint_4"] = math.radians(270)
        pick_full_configuration["robot12_joint_5"] = math.radians(85)
        pick_full_configuration["robot12_joint_6"] = math.radians(-6)
        return pick_full_configuration
    
    @property
    def inter_configuration(self):
        inter_full_configuration = self.current_configuration.copy()
        inter_full_configuration["bridge1_joint_EA_X"] = 9.3
        inter_full_configuration["robot12_joint_EA_Y"] = -1.2
        inter_full_configuration["robot12_joint_EA_Z"] = -4.9
        inter_full_configuration["robot12_joint_1"] = math.radians(90)
        inter_full_configuration["robot12_joint_2"] = math.radians(-30)
        inter_full_configuration["robot12_joint_3"] = math.radians(-35)
        inter_full_configuration["robot12_joint_4"] = math.radians(180)
        inter_full_configuration["robot12_joint_5"] = math.radians(25)
        inter_full_configuration["robot12_joint_6"] = math.radians(-90)
        return inter_full_configuration
    
    @property
    def flip_configuration(self):
        flip_full_configuration = self.current_configuration.copy()
        flip_full_configuration["bridge1_joint_EA_X"] = 11.1
        flip_full_configuration["robot12_joint_EA_Y"] = -1.1
        flip_full_configuration["robot12_joint_EA_Z"] = -4.9
        flip_full_configuration["robot12_joint_1"] = math.radians(0)
        flip_full_configuration["robot12_joint_2"] = math.radians(-55)
        flip_full_configuration["robot12_joint_3"] = math.radians(55)
        flip_full_configuration["robot12_joint_4"] = math.radians(180)
        flip_full_configuration["robot12_joint_5"] = math.radians(90)
        flip_full_configuration["robot12_joint_6"] = math.radians(-0)
        return flip_full_configuration

    @property
    def PUS_configuration(self):
        PUS_full_configuration = self.current_configuration.copy()
        PUS_full_configuration["bridge1_joint_EA_X"] = 10
        PUS_full_configuration["robot12_joint_EA_Y"] = -5
        PUS_full_configuration["robot12_joint_EA_Z"] = -4
        PUS_full_configuration["robot12_joint_1"] = math.radians(90)
        PUS_full_configuration["robot12_joint_2"] = math.radians(-30)
        PUS_full_configuration["robot12_joint_3"] = math.radians(-35)
        PUS_full_configuration["robot12_joint_4"] = math.radians(180)
        PUS_full_configuration["robot12_joint_5"] = math.radians(25)
        PUS_full_configuration["robot12_joint_6"] = math.radians(-90)
        return PUS_full_configuration
    
    @property
    def AT_configuration(self):
        AT_full_configuration = self.current_configuration.copy()
        AT_full_configuration["bridge1_joint_EA_X"] = 10
        AT_full_configuration["robot12_joint_EA_Y"] = -6
        AT_full_configuration["robot12_joint_EA_Z"] = -6
        AT_full_configuration["robot12_joint_1"] = math.radians(-90) #might need to be -90, this was the original value
        AT_full_configuration["robot12_joint_2"] = math.radians(-30)
        AT_full_configuration["robot12_joint_3"] = math.radians(-35)
        AT_full_configuration["robot12_joint_4"] = math.radians(180)
        AT_full_configuration["robot12_joint_5"] = math.radians(25)
        AT_full_configuration["robot12_joint_6"] = math.radians(-90)
        return AT_full_configuration

    @property
    def global_constraints(self):
        constraints = []
        # constraints.append(JointConstraint('robot12_joint_1', math.radians(90), math.radians(85), math.radians(175), 0.5))
        # constraints.append(JointConstraint('robot12_joint_2', math.radians(-55), math.radians(200), math.radians(30), 1.0))
        # constraints.append(JointConstraint('robot12_joint_3', math.radians(55), math.radians(10), math.radians(210), 1.0))
        # constraints.append(JointConstraint('robot12_joint_4', math.radians(180), math.radians(270), math.radians(270), 1.0))
        # constraints.append(JointConstraint('robot12_joint_5', math.radians(90), math.radians(25), math.radians(180), 0.5))
        # constraints.append(JointConstraint('robot12_joint_6', math.radians(-90), math.radians(180), math.radians(0), 1.0))
        # constraints.append(JointConstraint('bridge1_joint_EA_X', 13, 2, 4, 1.0))
        # constraints.append(JointConstraint('robot12_joint_EA_Y', -7, 4.5, 5, 1.0))
        # constraints.append(JointConstraint('robot12_joint_EA_Z', -4, 2, 1, 0.7))
        # BV = BoundingVolume.from_mesh(Mesh.from_stl(compas.get("C:\\Users\\paulj\\github\\fall_demo_2025\\data\\models\\bounding_volume.stl")))
        # constraints.append(PositionConstraint('robot12_link_6', BV, 1.0))
        return constraints

    # =========================================================================
    # TIMBER PROCESS LOGIC
    # =========================================================================
    
    def pick_and_place_element(self, element_guid, timber_model):
        element = timber_model.element_by_guid(element_guid)
        print("Picking and placing element:", element.guid)
        trajectories = []

        grasp_frame, element_at_frame, element_geometry_at = self.calculate_element_at_frame(element)
        
        element_pickup_frame = self.calculate_element_pickup_frame(grasp_frame, element_at_frame)

        # 1. Approach pickpoint
        print("getting element approach trajectory to pickpoint")
        approach_frame = self.get_approach_frame(element_pickup_frame, approach_distance=0.5)
        trajectories.append(self.get_motion_to_frame(approach_frame))

        # 2. Pick Element at pickpoint
        print("getting element pick trajectory at pickpoint")
        trajectories.append(self.get_cartesian_trajectory([element_pickup_frame]))

        # Prepare mesh for attachment
        element_mesh_at = element_geometry_at.to_viewmesh()[0]
        # adjusted_grasp_frame = element_grasp_frame.copy()
        # adjusted_grasp_frame.point.z -= 0.08  # Account for gripper offset
        self.attach_workpiece(str(element.guid), element_mesh_at, grasp_frame, attached_to_tool="GripperLong")

        # 3. Retract from pickpoint
        print("getting element retract trajectory at pickpoint")
        trajectories.append(self.get_retract_trajectory(retract_distance=0.5))

        # # 4. Safe AT 
        # print("getting element safe trajectory to AT")
        # safe_at_configuration = self.AT_configuration.copy()
        # safe_at_configuration["robot12_joint_EA_Z"] += 1.0
        # trajectories.append(self.get_motion_to_configuration(safe_at_configuration))

        # 5. Approach AT
        print("getting element approach trajectory to AT")
        element_at_approach_frame = self.get_approach_frame(element_at_frame, approach_distance=0.5)
        trajectories.append(self.get_motion_to_frame(element_at_approach_frame))

        # 6. Place at AT
        print("getting element place trajectory at AT")
        # Overriding default options to disable collision avoidance for the final placement
        trajectories.append(self.get_cartesian_trajectory([element_at_frame], avoid_collisions=False))
        
        self.detach_workpiece(str(element.guid))

        # 7. Retract from AT
        print("getting element retract trajectory at AT")
        trajectories.append(self.get_retract_trajectory(retract_distance=0.5, avoid_collisions=False))

        # 8. Return to safe configuration
        print("getting trajectory back to safe configuration")
        trajectories.append(self.get_motion_to_configuration(self.safe_configuration))

        json_dump(trajectories, "C:\\Users\\paulj\\Downloads\\element_trajs.json")
        return trajectories

    # =========================================================================
    # TIMBER MATH & GEOMETRY HELPERS
    # =========================================================================

    def get_stock_pick_x(self, stock_length, consoles_positions):
        """Calculates optimal X offset for picking stock to avoid console collisions."""
        gripper_width = 0.1
        margin = 0.02
        base_g1 = stock_length / 2.0 - 0.3
        base_g2 = stock_length / 2.0 + 0.2
        consoles_domains = [[c/1000, c/1000 + 0.14] for c in consoles_positions]
        consoles_domains.extend([[1.085, 1.165], [2.512, 2.592]]) 

        if self.is_valid(base_g1, base_g2, consoles_domains, 0):
            return (base_g1 + 0.3)
        
        candidates = []
        for c_start, c_end in consoles_domains:
            candidates.extend([
                (c_start - margin - gripper_width) - base_g1,
                (c_end + margin) - base_g1,
                (c_start - margin - gripper_width) - base_g2,
                (c_end + margin) - base_g2
            ])

        candidates.sort(key=abs)
        for shift in candidates:
            if self.is_valid(base_g1, base_g2, consoles_domains, shift):
                return ((base_g1 + shift) + 0.3)
        raise ValueError("Impossible to fit grippers")

    def is_valid(self, base_g1, base_g2, consoles_domains, shift_val):
        g1_start, g2_start = base_g1 + shift_val, base_g2 + shift_val
        g1_end, g2_end = g1_start + 0.1, g2_start + 0.1
        
        for c_start, c_end in consoles_domains:
            if (g1_start < c_end and g1_end > c_start) or (g2_start < c_end and g2_end > c_start):
                return False
        return True
    
    def get_element_grasp_frame(self, element, pick_flag):
        gpx = element.attributes.get("gripper_position")
        if gpx is None:
            gpx = element.blank_length / 2

        if int(element.blank.ysize) == int(element.blank.zsize) == 140 or int(element.blank.ysize) == 280 or int(element.blank.zsize) == 280:
            if pick_flag:
                point_y = -(element.blank.ysize / 2) + 30
            else:
                point_y = (element.blank.ysize / 2) - 30
        else:
            point_y = 0

        return Frame(Point(gpx, point_y, element.height / 2), Vector.Xaxis(), -Vector.Yaxis()).scaled(0.001)
    
    def calculate_element_pickup_frame(self, grasp_frame, element_at_frame):
        dist = 99999999999999999999999999
        closest_ps_T = None
        closest_ps_frame = None
        for ps_T, ps_frame in zip(self.ps_Ts, self.ps_frames):
            ps_at_dist = element_at_frame.point.distance_to_point(ps_frame.point)
            if ps_at_dist < dist:
                dist = ps_at_dist
                closest_ps_T = ps_T
                closest_ps_frame = ps_frame
        element_pickup_frame = grasp_frame.transformed(closest_ps_T)      
        return element_pickup_frame
    
    def get_inside_plate_thickness(self, element):
        siblings = element.parent.children
        sibling_plate = next((s for s in siblings if s.name == "inside_plate"), None)
        # sibling_plate = next((s for s in self.model.plates if s.name == "inside_plate"), None)
        return sibling_plate.thickness if sibling_plate else 0

    def calculate_element_at_frame(self, element, grasp_frame=None):
        if not grasp_frame:
            grasp_frame = element.attributes.get("grasp_frame") 
        e_at_frame = grasp_frame.transformed(self.at_T*element.attributes.get("parent_T"))
        grasp_frame.transform(element.transformation_to_local())
        element_geometry_at = element.geometry.transformed(element.transformation_to_local())
        # ip_thickness = self.get_inside_plate_thickness(element)
        # e_at_frame.translate(e_at_frame.zaxis * (ip_thickness * 0.001))  # OFFSET FOR INSIDE PLATE THICKNESS
        # e_at_frame.translate(e_at_frame.zaxis * 0.08)  # DEPTH OFFSET FOR GRIPPER 60 WIDTH
        return grasp_frame, e_at_frame, element_geometry_at