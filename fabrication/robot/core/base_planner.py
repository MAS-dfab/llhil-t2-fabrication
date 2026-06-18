import math
import time

from compas.geometry import Frame
from compas.geometry import Transformation
from compas_fab.backends import MoveItPlanner
from compas_fab.robots import ConfigurationTarget, JointConstraint
from compas_fab.robots import FrameTarget
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import RigidBody
from compas_fab.robots import RigidBodyState
from compas_fab.robots import TargetMode
from compas_fab.robots import ToolState
from compas_fab.viewer import WorkpieceManager
from compas_robots import ToolModel
from compas_rrc import RosClient
from compas.datastructures import Mesh
from config.live_config import get_robot_data


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
            "max_step": 0.01,
            "path_constraints": self.global_constraints
        }
        self.enforce_joint_limits()
        

    def enforce_joint_limits(self):
        """Utility to enforce joint limits on a given configuration."""
        print("Enforcing joint limits with buffer...")
        robot_model = self.robot_cell.robot_model
        for joint in robot_model.joints:
            if joint.type == 0:  # Only apply to revolute joints
                joint.limit.lower += math.radians(4)  # Add small buffer to avoid exact limits
                joint.limit.upper -= math.radians(4)
        self.robot_cell.robot_model = robot_model
                         
    @property
    def current_configuration(self):
        """Returns the current full configuration of the robot state."""
        return self.state.robot_configuration
        
    def relauch_client_and_planner(self):
        """Utility to relaunch the ROS client and MoveIt planner, useful for resetting state."""
        self.client.close()
        self.client.terminate()
        time.sleep(1)  # Ensure clean shutdown
        self.client.run()
        self.planner = MoveItPlanner(self.client)
        self.planner.set_robot_cell(self.robot_cell)
        self.planner.set_robot_cell_state(self.state)
        print("ROS client and MoveIt planner relaunched.")

    def update_configuration_from_live_data(self):
        data = get_robot_data()
        if data is None:
            print("Failed to fetch live robot data. Using current configuration.")
            return self.current_configuration
        else:
            live_config = self.current_configuration.copy()
            live_config["bridge1_joint_EA_X"] = data[0] * 0.001
            live_config["robot12_joint_EA_Y"] = data[1] * 0.001
            live_config["robot12_joint_EA_Z"] = data[2] * 0.001
            live_config["robot12_joint_1"] = math.radians(data[3])
            live_config["robot12_joint_2"] = math.radians(data[4])
            live_config["robot12_joint_3"] = math.radians(data[5])
            live_config["robot12_joint_4"] = math.radians(data[6])
            live_config["robot12_joint_5"] = math.radians(data[7])
            live_config["robot12_joint_6"] = math.radians(data[8])
            self.state.robot_configuration.merge(live_config)
            return live_config

    def get_current_end_frame(self):
        """Returns the current forward kinematics frame of the tool."""
        return self.planner.forward_kinematics(self.state, TargetMode.TOOL, self.group)

    def get_fk_from_config(self, configuration):
        """Calculates the forward kinematics frame for a given configuration."""
        temp_state = self.state.copy()
        temp_state.robot_configuration = configuration
        return self.planner.forward_kinematics(temp_state, TargetMode.TOOL, self.group)
    
    def get_ik_from_frame(self, target_frame, path_constraints=[]):
        """Calculates the inverse kinematics configuration for a given target frame."""
        frame_target = FrameTarget(target_frame, target_mode=TargetMode.TOOL)
        options = {
            "return_full_configuration": True,
            "allow_collisions": False,
            "constraints": path_constraints
        }
        ik_config = self.planner.inverse_kinematics(frame_target, self.state, self.group, options=options)
        try:
            # ik_config = next(ik_iterator)
            return ik_config
        except StopIteration:
            print("No IK solution found for the given frame.")

    def get_constrained_ik_from_frame(self, target_frame):
        frame_target = FrameTarget(target_frame, target_mode=TargetMode.TOOL)
        j_constraints = [
            JointConstraint('robot12_joint_2', math.radians(-40), math.radians(15), math.radians(-15)),
            JointConstraint('robot12_joint_3', math.radians(40), math.radians(15), math.radians(-15)),
            JointConstraint('robot12_joint_6', math.radians(0), math.radians(10), math.radians(-10)),
        ]
        ik_options = {"constraints": j_constraints, "allow_collisions": False, "return_full_configuration": True, "max_results": 10000}
        approach_config = self.planner.inverse_kinematics(frame_target, self.state, self.group, options=ik_options)
        return approach_config

    def update_state_from_trajectory(self, trajectory, grp="robot12_eaXYZ"):
        """Updates the internal robot state to match the end of a trajectory."""
        if not trajectory or not trajectory.points:
            return
            
        jtp = trajectory.points[-1]
        configuration = self.planner._build_configuration(
            jtp.joint_values, 
            trajectory.joint_names, 
            grp, 
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

    def get_approach_frame(self, target_frame, approach_distance=1.5, vector=None):
        """Generates an approach frame backed off along the Z-axis of the target frame."""
        if vector:
            return target_frame.translated(vector * -approach_distance)
        else:
            return target_frame.translated(target_frame.zaxis * -approach_distance)

    def get_cartesian_trajectory(self, frames_list, avoid_collisions=True, planning_group=None, path_constraints=None):
        """Plans a Cartesian trajectory through a list of frames."""
        current_frame = self.get_current_end_frame()
        frames_list.insert(0, current_frame)
        
        waypoints = FrameWaypoints(frames_list, TargetMode.TOOL)
        self.state.robot_configuration = self.current_configuration
        plan_options = self.default_options.copy()
        # plan_options["path_constraints"] = list(self.default_options["path_constraints"])
        plan_options["avoid_collisions"] = avoid_collisions
        if path_constraints:
            plan_options["path_constraints"].extend(path_constraints)

        # plan_options["path_constraints"].append(JointConstraint('robot12_joint_2', math.radians(0), math.radians(85), math.radians(85), 1.0))

        trajectory = None
        tool_moved_to_planning_group = False
        try:
            if planning_group:
                self.state.set_tool_attached_to_group(self.state.get_attached_tool_id(self.group), planning_group, touch_links=["robot12_link_6"])
                tool_moved_to_planning_group = True
                trajectory = self.planner.plan_cartesian_motion(waypoints, self.state, group=planning_group, options=plan_options)
            else:
                trajectory = self.planner.plan_cartesian_motion(waypoints, self.state, group=self.group, options=plan_options)
            print(f"Cartesian trajectory planned. Fraction: {trajectory.fraction}")
            self.trajectory_list.append(trajectory)
            if planning_group:
                self.update_state_from_trajectory(trajectory, grp=planning_group)
            else:
                self.update_state_from_trajectory(trajectory)
        except Exception as e:
            print(f"Cartesian planning failed: {e}")
        finally:
            if tool_moved_to_planning_group:
                self.state.set_tool_attached_to_group(self.state.get_attached_tool_id(planning_group), self.group, touch_links=["robot12_link_6"])
        return trajectory

    def get_motion_to_frame(self, target_frame, planning_group=None, path_constraints=None, update_state=True, update_trajectory_list=True):
        """Plans a free-space (non-Cartesian) motion to a target frame."""
        frame_target = FrameTarget(target_frame, target_mode=TargetMode.TOOL)
        self.state.robot_configuration = self.current_configuration
        plan_options = {
            "allowed_planning_time": 10, 
            "num_planning_attempts": 50,
            "max_steps": 0.01,
            "path_constraints": self.global_constraints
            }
        if path_constraints:
            plan_options["path_constraints"].extend(path_constraints)

        trajectory = None
        try:
            if planning_group:
                trajectory = self.planner.plan_motion(frame_target, self.state, group=planning_group, options=plan_options)
            else:
                trajectory = self.planner.plan_motion(frame_target, self.state, group=self.group, options=plan_options)
            print(f"Free-space motion to frame planned. Fraction: {trajectory.fraction}")
            if update_trajectory_list:
                self.trajectory_list.append(trajectory)
            if update_state:
                self.update_state_from_trajectory(trajectory)
        except Exception as e:
            print(f"Free-space planning failed: {e}")
        return trajectory

    def get_motion_to_configuration(self, target_configuration, planning_group=None, update_state=True, update_trajectory_list=True):
        """Plans a free-space motion to a specific joint configuration."""
        def_tol = ConfigurationTarget.generate_default_tolerances(target_configuration, 0.01, math.radians(0.1))
        config_target = ConfigurationTarget(target_configuration, def_tol[0], def_tol[1])

        self.state.robot_configuration = self.current_configuration
        plan_options = {
            "allowed_planning_time": 10, 
            "num_planning_attempts": 100,
            "max_steps": 0.01,
            # "path_constraints": self.global_constraints
            "path_constraints": []
            }

        trajectory = None
        try:
            if planning_group:
                trajectory = self.planner.plan_motion(config_target, self.state, group=planning_group, options=plan_options)
            else:
                trajectory = self.planner.plan_motion(config_target, self.state, group=self.group, options=plan_options)
            print(f"Motion to configuration planned. Fraction: {trajectory.fraction}")
            if update_trajectory_list:
                self.trajectory_list.append(trajectory)
            if update_state:
                self.update_state_from_trajectory(trajectory)
        except Exception as e:
            print(f"Configuration planning failed: {e}")
        return trajectory

    def get_retract_trajectory(self, retract_distance=0.5, z_axis_only=False, avoid_collisions=True, planning_group=None):
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
        r_options["path_constraints"] = self.global_constraints
        trajectory = None
        try:
            if planning_group:
                trajectory = self.planner.plan_cartesian_motion(waypoints, self.state, group=planning_group, options=r_options)
            else:
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

    def attach_workpiece(self, name, viz_mesh, col_mesh, grasp_frame, attached_to_tool="gripper"):
        """
        Generic method to attach a mesh to the robot's tool for pick and place.
        """
        print(f"Attaching workpiece: {name}")

        tool_viz_mesh = Mesh.from_stl("fabrication\\data\\gripper\\GripperMedium_viz.stl")
        tool_viz_mesh.transform(Transformation.from_frame(grasp_frame))
        tool_col_mesh = Mesh.from_stl("fabrication\\data\\gripper\\GripperMedium_col.stl")
        tool_col_mesh.transform(Transformation.from_frame(grasp_frame))

        
        beam_tool_viz_mesh = viz_mesh.copy()
        beam_tool_col_mesh = col_mesh.copy()

        beam_tool_viz_mesh.join(tool_viz_mesh)
        beam_tool_col_mesh.join(tool_col_mesh)

        # beam_tool_viz_mesh = [mesh, tool_viz_mesh]
        rigid_body = RigidBody(beam_tool_viz_mesh, beam_tool_col_mesh)
        self.robot_cell.rigid_body_models[name] = rigid_body
        
        # Calculate attachment frame relative to the tool
        grasp_frame = grasp_frame.translated(grasp_frame.zaxis * -0.1)  # Offset to account for gripper length
        T_object_relative_to_tool = Transformation.from_frame(grasp_frame).inverse()
        moveit_attachment_frame = Frame.from_transformation(T_object_relative_to_tool)
        # moveit_attachment_frame = moveit_attachment_frame.translated(moveit_attachment_frame.zaxis * 0.1)
        
        self.state.rigid_body_states[name] = RigidBodyState(
            frame=None, 
            attached_to_tool="schunk", 
            attachment_frame=moveit_attachment_frame,
            touch_links=["robot12_link_6", "robot12_link_5"]
        )
        self.planner.set_robot_cell(self.robot_cell)
        self.planner.set_robot_cell_state(self.state)

        # Viewer caching
        self.workpiece_manager.add_element(
            name=name, 
            mesh=beam_tool_viz_mesh, 
            attach_time=self._get_current_trajectory_time(), 
            attachment_frame=moveit_attachment_frame
        )
        print(f"Workpiece '{name}' attached to tool '{attached_to_tool}'")

    def detach_workpiece(self, name, frame):
        """Detaches a workpiece from the robot."""
        if name in self.robot_cell.rigid_body_models:
            rbs = self.state.rigid_body_states.get(name)

            # Compute the rigid body's world frame by composing the tool placement frame
            # with the stored attachment offset: T_world_body = T_world_tool * T_tool_body
            if frame is not None and rbs.attachment_frame is not None:
                T_world_tool = Transformation.from_frame(frame)
                T_tool_body = Transformation.from_frame(rbs.attachment_frame)
                place_body_frame = Frame.from_transformation(T_world_tool * T_tool_body)
            else:
                place_body_frame = frame

            rbs.frame = place_body_frame
            rbs.attached_to_tool = None
            rbs.attachment_frame = None
            rbs.touch_links = []
            self.planner.set_robot_cell(self.robot_cell)
            self.planner.set_robot_cell_state(self.state)
            
            self.workpiece_manager.drop_element(name, self._get_current_trajectory_time())
            print(f"Workpiece '{name}' detached.")
        else:
            print(f"Warning: Workpiece '{name}' not found in attached bodies.")