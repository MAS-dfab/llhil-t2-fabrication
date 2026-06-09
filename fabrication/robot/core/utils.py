from compas_robots import Configuration
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint
from compas_fab.robots.time_ import Duration

def combine_trajectories(trajectories: list[JointTrajectory]) -> JointTrajectory:
    trajectories = [t for t in trajectories if t is not None and t.points]

    if not trajectories:
        return JointTrajectory()

    first_traj = trajectories[0]

    # 1. Build a master map of all unique joint names, types, and INITIAL positions.
    joint_types_map = {}
    current_positions = {}

    if first_traj.start_configuration and first_traj.start_configuration.joint_names:
        for name, val, jtype in zip(
            first_traj.start_configuration.joint_names,
            first_traj.start_configuration.joint_values,
            first_traj.start_configuration.joint_types,
        ):
            joint_types_map[name] = jtype
            current_positions[name] = val

    for traj in trajectories:
        first_point = traj.points[0]
        for name, jtype in zip(traj.joint_names, first_point.joint_types):
            if name not in joint_types_map:
                joint_types_map[name] = jtype
                idx = traj.joint_names.index(name)
                current_positions[name] = first_point.joint_values[idx]

    master_joint_names = list(joint_types_map.keys())

    start_vals = [current_positions[name] for name in master_joint_names]
    start_types = [joint_types_map[name] for name in master_joint_names]
    combined_start_config = Configuration(start_vals, start_types, master_joint_names)

    # 3. Process points and pad missing joint states dynamically
    combined_points = []
    total_planning_time = 0.0
    time_offset = Duration(0, 0)
    all_fractions_complete = True
    NANO_TO_SEC = int(1e9) # Explicit constant for division

    for i, traj in enumerate(trajectories):
        segment_duration = traj.points[-1].time_from_start

        for j, point in enumerate(traj.points):
            # FIX: Skip the first point of subsequent trajectories 
            # to avoid duplicating the timestamp of the previous trajectory's last point (dt=0)
            if i > 0 and j == 0:
                continue

            for name, val in zip(traj.joint_names, point.joint_values):
                current_positions[name] = val

            ordered_values = []
            ordered_types = []
            ordered_velocities = []
            ordered_accelerations = []
            ordered_efforts = []

            for name in master_joint_names:
                ordered_values.append(current_positions[name])
                ordered_types.append(joint_types_map[name])

                if name in traj.joint_names:
                    idx = traj.joint_names.index(name)
                    ordered_velocities.append(point.velocities[idx])
                    ordered_accelerations.append(point.accelerations[idx])
                    ordered_efforts.append(point.effort[idx])
                else:
                    ordered_velocities.append(0.0)
                    ordered_accelerations.append(0.0)
                    ordered_efforts.append(0.0)

            # BULLETPROOF TIME CALCULATION
            # Safely capture any nanosecond overflow into seconds using divmod
            print("Calculating new time_from_start for point:", point.time_from_start)
            total_nsecs = time_offset.nsecs + point.time_from_start.nsecs
            carry_secs, final_nsecs = divmod(total_nsecs, NANO_TO_SEC)
            
            new_secs = time_offset.secs + point.time_from_start.secs + carry_secs
            new_time = Duration(new_secs, final_nsecs)
            print(new_time)

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

        # Apply same bulletproof logic to the rolling offset update
        total_offset_nsecs = time_offset.nsecs + segment_duration.nsecs
        carry_offset_secs, final_offset_nsecs = divmod(total_offset_nsecs, NANO_TO_SEC)
        
        new_offset_secs = time_offset.secs + segment_duration.secs + carry_offset_secs
        time_offset = Duration(new_offset_secs, final_offset_nsecs)

        if traj.planning_time is not None:
            total_planning_time += traj.planning_time

        if traj.fraction is None or traj.fraction < 1.0:
            all_fractions_complete = False

    # 4. Package the unified trajectory
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