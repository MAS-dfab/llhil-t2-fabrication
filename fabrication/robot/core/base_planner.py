import math

from compas.geometry import Frame
from compas.geometry import Transformation
from compas_fab.backends import MoveItPlanner
from compas_fab.robots import ConfigurationTarget
from compas_fab.robots import FrameTarget
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import RigidBody
from compas_fab.robots import RigidBodyState
from compas_fab.robots import TargetMode
from compas_fab.robots import ToolState
from compas_fab.viewer import WorkpieceManager
from compas_robots import ToolModel
from compas_rrc import RosClient


class BaseRobotPlanner():
    """
    A generic trajectory planner for compas_fab and ROS/MoveIt.
    Handles basic movements (Cartesian and Free-Space), kinematics, and attachment management.
    """
    def __init__(self, group):
        super().__init__()
        # ROS and Planner Setup
        self.client = RosClient()
        self.client.run()

        self.planner = MoveItPlanner(self.client)
        self.robot_cell = self.client.load_robot_cell(load_geometry=True)
        print(self.robot_cell.robot_model)
        self.state = self.robot_cell.default_cell_state()
        
        # Configuration
        self.group = group or self.robot_cell.main_group_name
        self.trajectory_list = []
        self.workpiece_manager = WorkpieceManager()
        
        # Default Options (Can be overridden by child classes)
        self.default_options = {
            "max_step": 0.1,
            "path_constraints": []
        }

    @property
    def current_configuration(self):
        """Returns the current full configuration of the robot state."""
        if len(self.trajectory_list) == 0:
            return self.safe_configuration
        else:
            full_config = self.state.robot_configuration
            return full_config

    def get_current_end_frame(self):
        """Returns the current forward kinematics frame of the tool."""
        return self.planner.forward_kinematics(self.state, TargetMode.TOOL, self.group)

    def get_fk_from_config(self, configuration):
        """Calculates the forward kinematics frame for a given configuration."""
        temp_state = self.state.copy()
        temp_state.robot_configuration = configuration
        return self.planner.forward_kinematics(temp_state, TargetMode.TOOL, self.group)
    
    def get_ik_from_frame(self, target_frame):
        """Calculates the inverse kinematics configuration for a given target frame."""
        frame_target = FrameTarget(target_frame, target_mode=TargetMode.TOOL)
        temp_state = self.state.copy()
        options = {
            "return_full_configuration": True,
            "allow_collisions": False
        }
        ik_config = self.planner.inverse_kinematics(frame_target, temp_state, self.group, options=options)
        try:
            # ik_config = next(ik_iterator)
            return ik_config
        except StopIteration:
            print("No IK solution found for the given frame.")

    def update_state_from_trajectory(self, trajectory):
        """Updates the internal robot state to match the end of a trajectory."""
        if not trajectory or not trajectory.points:
            return
            
        jtp = trajectory.points[-1]
        configuration = self.planner._build_configuration(
            jtp.joint_values, 
            trajectory.joint_names, 
            self.group, 
            return_full_configuration=False, 
            start_configuration=trajectory.start_configuration
        )
        self.state.robot_configuration.merge(configuration)
        print("RobotCellState updated from trajectory.")

    def _get_current_trajectory_time(self):
        """Calculates the total elapsed time of all currently planned trajectories."""
        total_time = 0.0
        for traj in self.trajectory_list:
            if traj and traj.points:
                pt_time = traj.points[-1].time_from_start
                total_time += pt_time.seconds if pt_time else 0.0
        return total_time

    # --- TRAJECTORY PLANNING ---

    def get_approach_frame(self, target_frame, approach_distance=1.5):
        """Generates an approach frame backed off along the Z-axis of the target frame."""
        return target_frame.translated(target_frame.zaxis * -approach_distance)

    def get_cartesian_trajectory(self, frames_list, avoid_collisions=True):
        """Plans a Cartesian trajectory through a list of frames."""
        current_frame = self.get_current_end_frame()
        frames_list.insert(0, current_frame)
        
        waypoints = FrameWaypoints(frames_list, TargetMode.TOOL)
        self.state.robot_configuration = self.current_configuration
        plan_options = self.default_options
        plan_options["avoid_collisions"] = avoid_collisions

        trajectory = None
        try:
            trajectory = self.planner.plan_cartesian_motion(waypoints, self.state, group=self.group, options=plan_options)
            print(f"Cartesian trajectory planned. Fraction: {trajectory.fraction}")
            self.trajectory_list.append(trajectory)
            self.update_state_from_trajectory(trajectory)
        except Exception as e:
            print(f"Cartesian planning failed: {e}")
        return trajectory

    def get_motion_to_frame(self, target_frame):
        """Plans a free-space (non-Cartesian) motion to a target frame."""
        frame_target = FrameTarget(target_frame, target_mode=TargetMode.TOOL)
        self.state.robot_configuration = self.current_configuration
        plan_options = {
            "allowed_planning_time": 10, 
            "num_planning_attempts": 20,
            "max_steps": 0.1,
            "path_constraints": []
            }

        trajectory = None
        try:
            trajectory = self.planner.plan_motion(frame_target, self.state, group=self.group, options=plan_options)
            print(f"Free-space motion to frame planned. Fraction: {trajectory.fraction}")
            self.trajectory_list.append(trajectory)
            self.update_state_from_trajectory(trajectory)
        except Exception as e:
            print(f"Free-space planning failed: {e}")
        return trajectory

    def get_motion_to_configuration(self, target_configuration):
        """Plans a free-space motion to a specific joint configuration."""
        def_tol = ConfigurationTarget.generate_default_tolerances(target_configuration, 0.01, math.radians(0.1))
        config_target = ConfigurationTarget(target_configuration, def_tol[0], def_tol[1])

        self.state.robot_configuration = self.current_configuration
        plan_options = {
            "allowed_planning_time": 30, 
            "num_planning_attempts": 100,
            "max_steps": 0.1,
            "path_constraints": []
            }

        trajectory = None
        try:
            trajectory = self.planner.plan_motion(config_target, self.state, group=self.group, options=plan_options)
            print(f"Motion to configuration planned. Fraction: {trajectory.fraction}")
            self.trajectory_list.append(trajectory)
            self.update_state_from_trajectory(trajectory)
        except Exception as e:
            print(f"Configuration planning failed: {e}")
        return trajectory

    def get_retract_trajectory(self, retract_distance=0.5, z_axis_only=False, avoid_collisions=True):
        """
        Plans a Cartesian retract motion. 
        If z_axis_only is True, it moves straight up in world Z. 
        Otherwise, it backs off along the tool's negative Z axis.
        """
        current_frame = self.get_current_end_frame()
        if z_axis_only:
            retract_frame = current_frame.translated([0, 0, retract_distance])
        else:
            retract_frame = current_frame.translated(current_frame.zaxis * -retract_distance)
        waypoints = FrameWaypoints([current_frame, retract_frame], TargetMode.TOOL)
        self.state.robot_configuration = self.current_configuration
        
        r_options = self.default_options.copy()
        r_options["avoid_collisions"] = avoid_collisions
        trajectory = None
        try:
            trajectory = self.planner.plan_cartesian_motion(waypoints, self.state, group=self.group, options=r_options)
            print(f"Retract trajectory planned. Fraction: {trajectory.fraction}")
            self.trajectory_list.append(trajectory)
            self.update_state_from_trajectory(trajectory)
        except Exception as e:
            print(f"Retract planning failed: {e}")
        return trajectory

    # --- TOOL & WORKPIECE MANAGEMENT ---

    def add_tool_to_robot(self, viz_mesh, col_mesh,  tool_frame, tool_name="gripper", connected_to="robot12_tool0"):
        """Attaches a static tool mesh to the robot."""
        tool_model = ToolModel(
            visual=viz_mesh,
            frame_in_tool0_frame=tool_frame,
            collision=col_mesh,
            name=tool_name,
            connected_to=connected_to,
        )

        self.robot_cell.tool_models[tool_model.name] = tool_model
        
        self.state.tool_states[tool_model.name] = ToolState(
            frame=None, 
            attached_to_group=self.group, 
            attachment_frame=Frame.worldXY(), 
            touch_links=["robot12_link_6"]
        )
        self.planner.set_robot_cell(self.robot_cell)
        self.planner.set_robot_cell_state(self.state)
        print(f"Tool '{tool_name}' attached to '{connected_to}'.")
        return tool_model

    def attach_workpiece(self, name, mesh, grasp_frame, attached_to_tool="gripper"):
        """
        Generic method to attach a mesh to the robot's tool for pick and place.
        """
        print(f"Attaching workpiece: {name}")
        rigid_body = RigidBody.from_mesh(mesh)
        self.robot_cell.rigid_body_models[name] = rigid_body
        
        # Calculate attachment frame relative to the tool
        T_object_relative_to_tool = Transformation.from_frame(grasp_frame).inverse()
        moveit_attachment_frame = Frame.from_transformation(T_object_relative_to_tool)
        
        self.state.rigid_body_states[name] = RigidBodyState(
            frame=None, 
            attached_to_tool=attached_to_tool, 
            attachment_frame=moveit_attachment_frame
        )
        self.planner.set_robot_cell(self.robot_cell)
        self.planner.set_robot_cell_state(self.state)

        # Viewer caching
        self.workpiece_manager.add_element(
            name=name, 
            mesh=mesh, 
            attach_time=self._get_current_trajectory_time(), 
            attachment_frame=moveit_attachment_frame
        )
        print(f"Workpiece '{name}' attached to tool '{attached_to_tool}'")

    def detach_workpiece(self, name):
        """Detaches a workpiece from the robot."""
        if name in self.robot_cell.rigid_body_models:
            self.robot_cell.rigid_body_models.pop(name)
            self.state.rigid_body_states.pop(name)
            self.planner.set_robot_cell(self.robot_cell)
            self.planner.set_robot_cell_state(self.state)
            
            self.workpiece_manager.drop_element(name, self._get_current_trajectory_time())
            print(f"Workpiece '{name}' detached.")
        else:
            print(f"Warning: Workpiece '{name}' not found in attached bodies.")