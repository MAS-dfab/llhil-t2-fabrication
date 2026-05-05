from compas_fab.robots import JointTrajectory
from compas_fab.robots.time_ import Duration


def combine_trajectories(trajectories: list[JointTrajectory]) -> JointTrajectory:
    """
    Combines a list of sequential trajectories into one.

    This function correctly recalculates the time_from_start
    for each point and handles potential None values in planning_time
    and fraction.
    """
    if not trajectories:
        return JointTrajectory()

    first_traj = trajectories[0]
    combined_points = []
    combined_joint_names = first_traj.joint_names
    combined_start_config = first_traj.start_configuration

    total_planning_time = 0.0
    time_offset = Duration(0, 0)
    all_fractions_complete = True

    for traj in trajectories:
        if not traj.points:
            continue

        if traj.joint_names != combined_joint_names:
            raise ValueError("Cannot combine trajectories with different joint_names.")

        segment_duration = traj.points[-1].time_from_start

        for point in traj.points:
            new_point = point.copy()

            new_secs = time_offset.secs + new_point.time_from_start.secs
            new_nsecs = time_offset.nsecs + new_point.time_from_start.nsecs

            new_point.time_from_start = Duration(new_secs, new_nsecs)

            combined_points.append(new_point)

        new_offset_secs = time_offset.secs + segment_duration.secs
        new_offset_nsecs = time_offset.nsecs + segment_duration.nsecs
        time_offset = Duration(new_offset_secs, new_offset_nsecs)

        if traj.planning_time is not None:
            total_planning_time += traj.planning_time

        if traj.fraction is None or traj.fraction < 1.0:
            all_fractions_complete = False

    combined_trajectory = JointTrajectory(
        trajectory_points=combined_points, 
        joint_names=combined_joint_names, 
        start_configuration=combined_start_config, 
        attributes=first_traj.attributes.copy()
    )

    combined_trajectory.planning_time = total_planning_time
    combined_trajectory.fraction = 1.0 if all_fractions_complete else None

    return combined_trajectory