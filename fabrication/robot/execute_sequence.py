import math

import compas_rrc as rrc
from compas.data import json_load

# ── Configuration ───────────────────────────────────────────────────────────────
SEQUENCE_FILE = "fabrication\\data\\fabrication_sequence.json"
TOOL          = "t_dummy"

# R11 stays at its home joint config; only X external axis tracks the trajectory
HOME_JOINTS_R11 = [-2.04, 90.27, -2.45, -0.05, -49.14, 0.0]
HOME_EXT_R11    = [17985.0, -2117.46, -4880.92, 0.0, 0.0, 0.0]

# Speeds (mm/s)
SPEED_FREE       = 500
SPEED_HOLD       = 200
SPEED_APPROACH_AT = 300
SPEED_PICK       = 20
SPEED_PLACE      = 20


# ── ROS ─────────────────────────────────────────────────────────────────────────

def connect_ros():
    ros = rrc.RosClient()
    ros.run()
    abb11 = rrc.AbbClient(ros, "/rob11")
    abb12 = rrc.AbbClient(ros, "/rob12")
    print("Connected to ROS.")
    return ros, abb11, abb12


def disconnect_ros(ros):
    try:
        ros.close()
        ros.terminate()
    except Exception:
        pass
    print("Disconnected from ROS.")


# ── Soft move ────────────────────────────────────────────────────────────────────

def soft_move_on(abb12):
    print("  Activating soft move...")
    abb12.send_and_wait(
        rrc.CustomInstruction(
            "r_A083_ActSoftMove",
            feedback_level=rrc.FeedbackLevel.DONE,
            float_values=[20, 80],
            string_values=["XYRZ"],
        ),
        timeout=5.0,
    )


def soft_move_off(abb12):
    print("  Deactivating soft move...")
    abb12.send_and_wait(
        rrc.CustomInstruction(
            "r_A083_DeactSoftMove",
            feedback_level=rrc.FeedbackLevel.DONE,
        ),
        timeout=5.0,
    )


# ── Trajectory execution ─────────────────────────────────────────────────────────
def _split_point(point):
    """Split a 9-DOF trajectory point into R11 ext-axes and R12 ext-axes + joints.

    joint_values layout (compas_fab group robot12_eaXYZ):
      [0] EA_X  (metres) → R11 external axis X
      [1] EA_Y  (metres) → R12 external axis Y
      [2] EA_Z  (metres) → R12 external axis Z
      [3..8]    robot12 joints (radians)
    """
    vals = point.joint_values
    ext_r11  = [vals[0] * 1000, HOME_EXT_R11[1], HOME_EXT_R11[2], 0.0, 0.0, 0.0]
    ext_r12  = [vals[1] * 1000, vals[2] * 1000, 0.0, 0.0, 0.0, 0.0]
    j_r12    = [math.degrees(j) for j in vals[3:]]
    return ext_r11, ext_r12, j_r12


def execute_trajectory(abb11, abb12, trajectory, speed):
    """Execute a JointTrajectory with R11 X and R12 Y/Z/joints in sync per point."""
    points = trajectory.points
    for i, pt in enumerate(points):
        ext_r11, ext_r12, j_r12 = _split_point(pt)
        is_last = (i == len(points) - 1)
        zone = rrc.Zone.FINE if is_last else rrc.Zone.Z100

        if is_last:
            # Wait for both robots to finish the final point
            m11 = rrc.MoveToJoints(HOME_JOINTS_R11, ext_r11, speed, zone)
            m12 = rrc.MoveToJoints(j_r12, ext_r12, speed, zone)
            m11.feedback_level = rrc.FeedbackLevel.DONE
            m12.feedback_level = rrc.FeedbackLevel.DONE
            f11 = abb11.send(rrc.MoveToJoints(HOME_JOINTS_R11, ext_r11, speed, zone))
            f12 = abb12.send(rrc.MoveToJoints(j_r12, ext_r12, speed, zone))
            f11.result(timeout=300.0)
            f12.result(timeout=300.0)
        else:
            # Fire-and-forget; controller buffers the move queue
            abb11.send(rrc.MoveToJoints(HOME_JOINTS_R11, ext_r11, speed, zone))
            abb12.send(rrc.MoveToJoints(j_r12, ext_r12, speed, zone))


# ── Main sequence ─────────────────────────────────────────────────────────────────
def execute_sequence(abb11, abb12, record):
    steps        = record["steps"]
    pickup_frame = record["pickup_frame"]
    place_frame  = record["place_frame"]

    # 1. Approach to pick
    print("\n[1/7] approach_to_pick")
    execute_trajectory(abb11, abb12, steps["approach_to_pick"], SPEED_FREE)

    # 2. Pick — soft-move MoveL to exact pickup frame
    print("\n[2/7] pick (soft-move MoveL)")
    soft_move_on(abb12)
    try:
        abb12.send_and_wait(rrc.MoveToFrame(pickup_frame, SPEED_PICK, rrc.Zone.FINE), timeout=300.0)
        abb12.send_and_wait(rrc.Stop(), timeout=1000.0)
    finally:
        soft_move_off(abb12)

    # 3. Retract from pick
    print("\n[3/7] retract_from_pick")
    execute_trajectory(abb11, abb12, steps["retract_from_pick"], SPEED_HOLD)

    # 4. Approach to assembly target
    print("\n[4/7] approach_to_AT")
    execute_trajectory(abb11, abb12, steps["approach_to_AT"], SPEED_APPROACH_AT)

    # 5. Place at assembly target — cartesian MoveL to exact place frame
    print("\n[5/7] place_at_AT (MoveL)")
    abb12.send_and_wait(rrc.MoveToFrame(place_frame, SPEED_PLACE, rrc.Zone.FINE), timeout=300.0)
    abb12.send_and_wait(rrc.Stop(), timeout=10.0)

    # 6. Retract from assembly target
    print("\n[6/7] retract_from_AT")
    execute_trajectory(abb11, abb12, steps["retract_from_AT"], SPEED_HOLD)

    # 7. Return to safe configuration
    print("\n[7/7] return_to_safe")
    execute_trajectory(abb11, abb12, steps["return_to_safe"], SPEED_FREE)

    print("\nSequence complete.")


# ── Entry point ───────────────────────────────────────────────────────────────────

def main():
    record = json_load(SEQUENCE_FILE)

    input("\nPress Enter to connect to ROS...")

    ros, abb11, abb12 = connect_ros()
    try:
        abb12.send_and_wait(rrc.SetTool(TOOL), timeout=5.0)
        abb11.send_and_wait(rrc.SetAcceleration(100, 100), timeout=5.0)
        abb12.send_and_wait(rrc.SetAcceleration(100, 100), timeout=5.0)
        abb11.send_and_wait(rrc.SetMaxSpeed(100, 250.0), timeout=5.0)
        abb12.send_and_wait(rrc.SetMaxSpeed(100, 250.0), timeout=5.0)

        input("\nRobot ready. Press Enter to start execution...")
        execute_sequence(abb11, abb12, record)

    except KeyboardInterrupt:
        print("\nAborted by user.")

    except Exception as e:
        print("ERROR during execution: {}".format(e))
        
    finally:
        disconnect_ros(ros)


if __name__ == "__main__":
    main()
