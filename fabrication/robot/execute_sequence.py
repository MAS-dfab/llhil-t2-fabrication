import math
import time

import compas_rrc as rrc
from compas.data import json_load

# ── Configuration ───────────────────────────────────────────────────────────────
SEQUENCE_FILE = "fabrication\\data\\fabrication_sequence.json"
TOOL          = "t_dummy"
WORKOBJECT    = "wobj0"

# R11 stays at its home joint config; only X external axis tracks the trajectory
HOME_JOINTS_R11 = [-2.04, 90.27, -2.45, -0.05, -49.14, 0.0]
HOME_EXT_R11    = [17985.0, -2117.46, -4880.92, 0.0, 0.0, 0.0]

# Speeds (mm/s)
SPEED_FREE       = 500
SPEED_HOLD       = 200
SPEED_APPROACH_AT = 300
SPEED_PICK       = 10
SPEED_PLACE      = 10


# ── ROS ─────────────────────────────────────────────────────────────────────────

def connect_ros():
    ros = rrc.RosClient()
    ros.run()
    abb11 = rrc.AbbClient(ros, "/r11")
    abb12 = rrc.AbbClient(ros, "/r12")
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
            float_values=[40, 90],
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
    ext_r12  = [vals[0] * 1000, vals[1] * 1000, vals[2] * 1000, 0.0, 0.0, 0.0]
    j_r12    = [math.degrees(j) for j in vals[3:]]
    return ext_r11, ext_r12, j_r12


def _ext_r12_from_last_point(trajectory):
    """Return R12 external axes [EA_X(mm), EA_Y(mm), EA_Z(mm), 0,0,0] from the last trajectory point."""
    _, ext_r12, _ = _split_point(trajectory.points[-1])
    return ext_r12

def _ext_r12_from_first_point(trajectory):
    """Return R12 external axes [EA_X(mm), EA_Y(mm), EA_Z(mm), 0,0,0] from the first trajectory point."""
    _, ext_r12, _ = _split_point(trajectory.points[0])
    return ext_r12

def _ext_r11_from_last_point(trajectory):
    """Return R11 external axes [EA_X(mm), HOME_Y, HOME_Z, 0,0,0] from the last trajectory point."""
    ext_r11, _, _ = _split_point(trajectory.points[-1])
    return ext_r11



def execute_trajectory(abb11, abb12, trajectory, speed):
    """Execute a JointTrajectory with R11 X and R12 Y/Z/joints in sync per point."""
    points = trajectory.points
    for i, pt in enumerate(points):
        ext_r11, ext_r12, j_r12 = _split_point(pt)
        is_last = (i == len(points) - 1)
        is_first = (i == 0)
        zone = rrc.Zone.FINE if is_last else rrc.Zone.Z100

        if is_first:
            # Send-and-wait for the first point so both robots reach a known state
            m11 = rrc.MoveToJoints(HOME_JOINTS_R11, ext_r11, speed, rrc.Zone.FINE)
            m12 = rrc.MoveToJoints(j_r12, ext_r12, speed, rrc.Zone.FINE)
            m11.feedback_level = rrc.FeedbackLevel.DONE
            m12.feedback_level = rrc.FeedbackLevel.DONE
            f11 = abb11.send(m11)
            f12 = abb12.send(m12)
            f11.result(timeout=60.0)
            f12.result(timeout=60.0)
        elif is_last:
            # Wait for both robots to finish the final point
            m11 = rrc.MoveToJoints(HOME_JOINTS_R11, ext_r11, speed, zone)
            m12 = rrc.MoveToJoints(j_r12, ext_r12, speed, zone)
            m11.feedback_level = rrc.FeedbackLevel.DONE
            m12.feedback_level = rrc.FeedbackLevel.DONE
            f11 = abb11.send(m11)
            f12 = abb12.send(m12)
            f11.result(timeout=60.0)
            f12.result(timeout=60.0)
        else:
            # Fire-and-forget; controller buffers the move queue
            abb11.send(rrc.MoveToJoints(HOME_JOINTS_R11, ext_r11, speed, zone))
            abb12.send(rrc.MoveToJoints(j_r12, ext_r12, speed, zone))


# ── Main sequence ─────────────────────────────────────────────────────────────────
def execute_sequence(abb11, abb12, record):
    steps        = record["steps"]
    pickup_frame = record["pickup_frame"]
    place_frame  = record["place_frame"]

    # Ensure soft move is off before starting
    soft_move_off(abb12)

    # 1. Approach to pick
    print("\n[1/7] approach_to_pick")
    execute_trajectory(abb11, abb12, steps["approach_to_pick"], SPEED_FREE)
    # small pause to ensure we're settled at the end of the approach trajectory

    # Corrective Cartesian move to the exact approach frame before descending
    print("  Corrective move to approach frame...")
    approach_frame = record.get("approach_frame")
    if approach_frame is not None:
        # app_ext_r12 = _ext_r12_from_last_point(steps["approach_to_pick"])
        app_ext_r11 = _ext_r11_from_last_point(steps["approach_to_pick"])
        if abb11.send_and_wait(rrc.GetJoints())[1][0] - app_ext_r11[0] > 2:  # sanity check to avoid large unexpected moves
            abb11.send_and_wait(rrc.MoveToJoints(HOME_JOINTS_R11, app_ext_r11, SPEED_HOLD, rrc.Zone.FINE), timeout=30.0)
        
        # small pause to ensure we're settled before the next move
        print("  Moving linearly to approach frame...", approach_frame)
        abb12.send_and_wait(rrc.MoveToFrame(approach_frame, SPEED_HOLD, rrc.Zone.FINE, motion_type=rrc.Motion.LINEAR), timeout=30.0)

    else:
        print("  WARNING: no approach_frame in record, skipping corrective move.")

    # 2. Pick
    print("\n[2/7] pick")
    time.sleep(1.2)  # small pause to ensure we're settled before the next move
    soft_move_on(abb12)
    abb12.send_and_wait(rrc.MoveToFrame(pickup_frame, SPEED_PICK, rrc.Zone.FINE, motion_type=rrc.Motion.LINEAR), timeout=30.0)
    soft_move_off(abb12)
    abb12.send(rrc.PrintText("Stopping here for a bit"))
    abb12.send_and_wait(rrc.Stop(), timeout=100.0)

        

    # 3. Retract from pick
    print("\n[3/7] retract_from_pick")
    pick_retract_frame = record.get("pick_retract_frame")
    if pick_retract_frame is not None:
        print("  MoveL to pick retract frame")
        abb12.send_and_wait(rrc.MoveToFrame(pick_retract_frame, SPEED_HOLD, rrc.Zone.FINE, motion_type=rrc.Motion.LINEAR), timeout=60.0)
    else:
        print("  WARNING: no pick_retract_frame in record, falling back to joint trajectory.")
        execute_trajectory(abb11, abb12, steps["retract_from_pick"], SPEED_HOLD)

    # 4. Approach to assembly target
    print("\n[4/7] approach_to_AT")
    execute_trajectory(abb11, abb12, steps["approach_to_AT"], SPEED_APPROACH_AT)

    # 5. Place at assembly target
    print("\n[5/7] place_at_AT")

    # MoveL to exact stored place frame
    place_ext_r12 = _ext_r12_from_last_point(steps["place_at_AT"])
    place_ext_r11 = _ext_r11_from_last_point(steps["place_at_AT"])
    m11 = rrc.MoveToJoints(HOME_JOINTS_R11, place_ext_r11, SPEED_PLACE, rrc.Zone.FINE)
    m12 = rrc.MoveToRobtarget(place_frame, place_ext_r12, SPEED_PLACE, rrc.Zone.FINE, motion_type=rrc.Motion.LINEAR)
    abb11.send_and_wait(m11, timeout=300.0)
    abb12.send_and_wait(m12, timeout=300.0)
    abb12.send(rrc.PrintText("Stopping here for a bit"))
    abb12.send_and_wait(rrc.Stop(), timeout=100.0)

    # 6. Retract from assembly target
    print("\n[6/7] retract_from_AT")
    place_retract_frame = record.get("place_retract_frame")
    if place_retract_frame is not None:
        print("  MoveL to place retract frame")
        # abb12.send_and_wait(rrc.MoveToFrame(place_retract_frame, SPEED_HOLD, rrc.Zone.FINE, motion_type=rrc.Motion.LINEAR), timeout=60.0)
        retract_ext_r12 = _ext_r12_from_first_point(steps["retract_from_AT"])
        abb12.send_and_wait(rrc.MoveToRobtarget(place_retract_frame, retract_ext_r12, SPEED_PLACE, rrc.Zone.FINE, motion_type=rrc.Motion.LINEAR))
    else:
        print("  WARNING: no place_retract_frame in record, falling back to joint trajectory.")
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
        abb12.send_and_wait(rrc.SetWorkObject(WORKOBJECT), timeout=5.0)
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
