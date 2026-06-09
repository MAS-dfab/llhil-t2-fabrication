from compas_robots import Configuration
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint
from compas_fab.robots.time_ import Duration


def combine_trajectories(trajectories: list[JointTrajectory]) -> JointTrajectory:
    """Combines a list of sequential trajectories with potentially different joint groups

    into one unified master trajectory. Missing joints in any given segment are held
    static at their last known positions with zero velocity/acceleration/effort.
    """
    # Filter out empty or None trajectories
    trajectories = [t for t in trajectories if t is not None and t.points]

    if not trajectories:
        return JointTrajectory()

    # 1. Build a master map of all unique joint names to their respective joint types
    joint_types_map = {}
    for traj in trajectories:
        # Map joint names to types using the first point's joint_types layout
        first_point = traj.points[0]
        for name, jtype in zip(traj.joint_names, first_point.joint_types):
            joint_types_map[name] = jtype

    master_joint_names = list(joint_types_map.keys())

    # 2. Track the latest position state of all joints to fill in gaps forward
    current_positions = {}

    # Initialize positions from the first trajectory's first point
    first_traj = trajectories[0]
    for name, val in zip(first_traj.joint_names, first_traj.points[0].joint_values):
        current_positions[name] = val

    # If any master joints are missing from the initial initialization, find their first occurrence
    for name in master_joint_names:
        if name not in current_positions:
            for traj in trajectories:
                if name in traj.joint_names:
                    idx = traj.joint_names.index(name)
                    current_positions[name] = traj.points[0].joint_values[idx]
                    break
            else:
                current_positions[name] = 0.0

    # 3. Rebuild the start_configuration to match the master joint structure
    combined_start_config = None
    if first_traj.start_configuration:
        start_vals = []
        start_types = []

        # Convert the original start configuration into a lookup dictionary
        orig_config = first_traj.start_configuration
        orig_vals_map = {}
        orig_types_map = {}
        if hasattr(orig_config, "joint_names") and orig_config.joint_names:
            for n, v, t in zip(orig_config.joint_names, orig_config.joint_values, orig_config.joint_types):
                orig_vals_map[n] = v
                orig_types_map[n] = t

        for name in master_joint_names:
            # Use value from original configuration, or fall back to our tracked positions
            start_vals.append(orig_vals_map.get(name, current_positions[name]))
            start_types.append(orig_types_map.get(name, joint_types_map[name]))

        combined_start_config = Configuration(start_vals, start_types, master_joint_names)

    # 4. Process points and pad missing joint states dynamically
    combined_points = []
    total_planning_time = 0.0
    time_offset = Duration(0, 0)
    all_fractions_complete = True

    for traj in trajectories:
        segment_duration = traj.points[-1].time_from_start

        for point in traj.points:
            # Update our tracker with any joint values explicitly provided by this point
            for name, val in zip(traj.joint_names, point.joint_values):
                current_positions[name] = val

            # Construct padded, ordered arrays matching master_joint_names
            ordered_values = []
            ordered_types = []
            ordered_velocities = []
            ordered_accelerations = []
            ordered_efforts = []

            for name in master_joint_names:
                ordered_values.append(current_positions[name])
                ordered_types.append(joint_types_map[name])

                if name in traj.joint_names:
                    # Joint exists in this segment: extract its dynamic states
                    idx = traj.joint_names.index(name)
                    ordered_velocities.append(point.velocities[idx])
                    ordered_accelerations.append(point.accelerations[idx])
                    ordered_efforts.append(point.effort[idx])
                else:
                    # Joint is missing in this segment: it is stationary
                    ordered_velocities.append(0.0)
                    ordered_accelerations.append(0.0)
                    ordered_efforts.append(0.0)

            # Recalculate time from start with duration offset
            new_secs = time_offset.secs + point.time_from_start.secs
            new_nsecs = time_offset.nsecs + point.time_from_start.nsecs
            new_time = Duration(new_secs, new_nsecs)

            # Instantiate unified point
            new_point = JointTrajectoryPoint(
                joint_values=ordered_values,
                joint_types=ordered_types,
                velocities=ordered_velocities,
                accelerations=ordered_accelerations,
                effort=ordered_efforts,
                time_from_start=new_time,
                joint_names=master_joint_names,
            )
            combined_points.append(new_point)

        # Update the rolling time offset for the next segment
        new_offset_secs = time_offset.secs + segment_duration.secs
        new_offset_nsecs = time_offset.nsecs + segment_duration.nsecs
        time_offset = Duration(new_offset_secs, new_offset_nsecs)

        if traj.planning_time is not None:
            total_planning_time += traj.planning_time

        if traj.fraction is None or traj.fraction < 1.0:
            all_fractions_complete = False

    # 5. Package the unified trajectory
    combined_trajectory = JointTrajectory(
        trajectory_points=combined_points,
        joint_names=master_joint_names,
        start_configuration=combined_start_config,
        attributes=first_traj.attributes.copy(),
    )

    combined_trajectory.planning_time = total_planning_time
    combined_trajectory.fraction = 1.0 if all_fractions_complete else None

    return combined_trajectory



# from compas_fab.robots import JointTrajectory
# from compas_fab.robots.time_ import Duration


# def combine_trajectories(trajectories: list[JointTrajectory]) -> JointTrajectory:
#     """
#     Combines a list of sequential trajectories into one.

#     This function correctly recalculates the time_from_start
#     for each point and handles potential None values in planning_time
#     and fraction.
#     """
#     # if not trajectories:
#     #     return JointTrajectory()
    
#     trajectories = [t for t in trajectories if t is not None and t.points]

#     first_traj = trajectories[0]
#     combined_points = []
#     combined_joint_names = first_traj.joint_names
#     combined_start_config = first_traj.start_configuration

#     total_planning_time = 0.0
#     time_offset = Duration(0, 0)
#     all_fractions_complete = True

#     for traj in trajectories:
#         if not traj.points:
#             continue

#         if traj.joint_names != combined_joint_names:
#             raise ValueError("Cannot combine trajectories with different joint_names.")

#         segment_duration = traj.points[-1].time_from_start

#         for point in traj.points:
#             new_point = point.copy()

#             new_secs = time_offset.secs + new_point.time_from_start.secs
#             new_nsecs = time_offset.nsecs + new_point.time_from_start.nsecs

#             new_point.time_from_start = Duration(new_secs, new_nsecs)

#             combined_points.append(new_point)

#         new_offset_secs = time_offset.secs + segment_duration.secs
#         new_offset_nsecs = time_offset.nsecs + segment_duration.nsecs
#         time_offset = Duration(new_offset_secs, new_offset_nsecs)

#         if traj.planning_time is not None:
#             total_planning_time += traj.planning_time

#         if traj.fraction is None or traj.fraction < 1.0:
#             all_fractions_complete = False

#     combined_trajectory = JointTrajectory(
#         trajectory_points=combined_points, 
#         joint_names=combined_joint_names, 
#         start_configuration=combined_start_config, 
#         attributes=first_traj.attributes.copy()
#     )

#     combined_trajectory.planning_time = total_planning_time
#     combined_trajectory.fraction = 1.0 if all_fractions_complete else None

#     return combined_trajectory