"""
Fabrication sequence executor — A083 Augmented-Timber
======================================================
Executes a pre-planned JointTrajectory sequence for one beam placement.

Trajectory joint layout (compas_fab group robot12_eaXYZ, 9-DOF):
  [0] bridge1_joint_EA_X   — metres  → R11 eax_a (shared X via PERS n_A083_SyncX)
  [1] robot12_joint_EA_Y   — metres  → R12 eax_b
  [2] robot12_joint_EA_Z   — metres  → R12 eax_c
  [3..8]                   — radians → R12 arm joints rax_1..6

Sync strategy (joint trajectories only):
  - Python sends paired instructions to BOTH R11 and R12 per waypoint.
  - R11 gets r_A083_MoveSyncR11 (X axis only); R12 gets r_A083_MoveSync (full).
  - Both tasks stay in the normal RRC dispatch loop — no blocking follower loop.
  - Cartesian steps (pick, place, retract) turn sync OFF; robots move
    independently via standard RRC instructions.
"""

import math
import time

import compas_rrc as rrc
from compas.data import json_load

# ── Configuration ────────────────────────────────────────────────────────────────
SEQUENCE_FILE = "fabrication\\data\\fabrication_sequence.json"
TOOL          = "t_dummy"
TOOL_LOADED   = "t_t2_2526_load"
WORKOBJECT    = "wobj0"

# R11 home — arm joints are static throughout; only X external axis moves.
HOME_JOINTS_R11 = [-2.04, 90.27, -2.45, -0.05, -49.14, 0.0]
HOME_EXT_R11    = [17985.0, -2117.46, -4880.92, 0.0, 0.0, 0.0]  # [X_mm, Y_mm, Z_mm, ...]

# Speeds (mm/s)
SPEED_FREE        = 500
SPEED_HOLD        = 200
SPEED_APPROACH_AT = 300
SPEED_PICK        = 10
SPEED_PLACE       = 10

# Zone values passed to RAPID r_RRC_RasZone (0 = fine, 100 = z100)
ZONE_FINE  = rrc.Zone.FINE
ZONE_BLEND = rrc.Zone.Z1

# Sync move timeout per waypoint
SYNC_TIMEOUT = 400.0


# ── ROS ──────────────────────────────────────────────────────────────────────────

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


# ── Soft move ─────────────────────────────────────────────────────────────────────

def soft_move_on(abb12):
    print("  Activating soft move...")
    abb12.send_and_wait(
        rrc.CustomInstruction(
            "r_A083_ActSoftMove",
            feedback_level=rrc.FeedbackLevel.DONE,
            float_values=[40, 90],
            string_values=["XYRZ"],
        ),
        timeout=400.0,
    )


def soft_move_off(abb12):
    print("  Deactivating soft move...")
    abb12.send_and_wait(
        rrc.CustomInstruction(
            "r_A083_DeactSoftMove",
            feedback_level=rrc.FeedbackLevel.DONE,
        ),
        timeout=400.0,   
    )


# ── Gantry sync ───────────────────────────────────────────────────────────────────

def sync_gantry_on(abb11, abb12, timeout=400.0):
    f11 = abb11.send(rrc.CustomInstruction(
        "r_A083_SyncGantryR11", feedback_level=rrc.FeedbackLevel.DONE))
    f12 = abb12.send(rrc.CustomInstruction(
        "r_A083_SyncGantry", feedback_level=rrc.FeedbackLevel.DONE))
    f12.result(timeout=timeout)
    f11.result(timeout=timeout)
    print("  Gantry sync ON.")


def sync_gantry_off(abb11, abb12, timeout=400.0):
    f11 = abb11.send(rrc.CustomInstruction(
        "r_A083_StopSyncR11", feedback_level=rrc.FeedbackLevel.DONE))
    f12 = abb12.send(rrc.CustomInstruction(
        "r_A083_StopSyncLoop", feedback_level=rrc.FeedbackLevel.DONE))
    f11.result(timeout=timeout)
    f12.result(timeout=timeout)
    print("  Gantry sync OFF.")


# ── Trajectory helpers ────────────────────────────────────────────────────────────

def _split_point(point):
    """
    Extract R12 full target from a 9-DOF trajectory point.

    Returns:
        x_mm  : X rail position (mm) — shared with R11 via n_A083_SyncX PERS
        j_r12 : [rax_1..6] degrees  — R12 arm joints
        y_r12 : R12 Y rail (mm)
        z_r12 : R12 Z rail (mm)
    """
    v = point.joint_values
    x_mm  = v[0] * 1000.0
    y_r12 = v[1] * 1000.0
    z_r12 = v[2] * 1000.0
    j_r12 = [math.degrees(j) for j in v[3:9]]
    return x_mm, j_r12, y_r12, z_r12



# ── Sync trajectory execution ──────────────────────────────────────────────────────

def execute_trajectory(abb11, abb12, trajectory, speed, zone_blend=ZONE_BLEND):
    points = trajectory.points
    n = len(points)
    for i, pt in enumerate(points):
        x_mm, j_r12, y_r12, z_r12 = _split_point(pt)
        is_last = (i == n - 1)
        zone = ZONE_FINE if is_last else zone_blend

        floats_r11 = [x_mm, float(speed), float(zone)]
        floats_r12 = [x_mm, float(speed), float(zone)] + j_r12 + [y_r12, z_r12]

        # Send R11 first so it reaches WaitSyncTask before R12
        abb11.send(rrc.CustomInstruction(
            "r_A083_MoveSyncR11",
            feedback_level=rrc.FeedbackLevel.NONE,
            float_values=floats_r11))

        if not is_last:
            abb12.send(rrc.CustomInstruction(
                "r_A083_MoveSync",
                feedback_level=rrc.FeedbackLevel.NONE,
                float_values=floats_r12))
        else:
            abb12.send_and_wait(rrc.CustomInstruction(
                "r_A083_MoveSync",
                feedback_level=rrc.FeedbackLevel.DONE,
                float_values=floats_r12),
                timeout=SYNC_TIMEOUT)



# ── Main sequence ─────────────────────────────────────────────────────────────────────────────


def execute_sequence(abb11, abb12, record):
    steps        = record["steps"]
    pickup_frame = record["pickup_frame"]
    place_frame  = record["place_frame"]

    soft_move_off(abb12)

    # ── 1. Approach to pick ──────────────────────────────────────────────────────────────────
    print("\n[1/7] approach_to_pick")
    sync_gantry_on(abb11, abb12, timeout=400.0)
    execute_trajectory(abb11, abb12, steps["approach_to_pick"], SPEED_FREE)
    sync_gantry_off(abb11, abb12, timeout=400.0)

    approach_frame = record.get("approach_frame")
    if approach_frame is not None:
        print("  Corrective move to approach frame...")
        abb12.send_and_wait(
            rrc.MoveToFrame(approach_frame, SPEED_HOLD, rrc.Zone.FINE,
                            motion_type=rrc.Motion.LINEAR),
            timeout=400.0)
    else:
        print("  WARNING: no approach_frame in record, skipping corrective move.")

    # ── 2. Pick ─────────────────────────────────────────────────────────────────────────────
    print("\n[2/7] pick")
    time.sleep(1.2)
    soft_move_on(abb12)
    abb12.send_and_wait(
        rrc.MoveToFrame(pickup_frame, SPEED_PICK, rrc.Zone.FINE,
                        motion_type=rrc.Motion.LINEAR),
        timeout=400.0)
    time.sleep(0.5)
    soft_move_off(abb12)
    # abb12.send(rrc.PrintText("Stopped after pick — press Play to continue"))
    time.sleep(2.0)
    abb12.send_and_wait(rrc.Stop(), timeout=400.0)

    # ── 3. Retract from pick ───────────────────────────────────────────────────────────────────
    print("\n[3/7] retract_from_pick")
    abb12.send_and_wait(rrc.SetTool(TOOL_LOADED), timeout=5.0)
    pick_retract_frame = record.get("pick_retract_frame")
    if pick_retract_frame is not None:
        print("  MoveL to pick retract frame")
        abb12.send_and_wait(
            rrc.MoveToFrame(pick_retract_frame, SPEED_HOLD, rrc.Zone.FINE,
                            motion_type=rrc.Motion.LINEAR),
            timeout=400.0)
    else:
        print("  Fallback: joint trajectory retract from pick")
        sync_gantry_on(abb11, abb12)
        execute_trajectory(abb11, abb12, steps["retract_from_pick"], SPEED_HOLD)
        sync_gantry_off(abb11, abb12)

    # ── 4. Approach to assembly target ─────────────────────────────────────────────────────
    print("\n[4/7] approach_to_AT")
    sync_gantry_on(abb11, abb12)
    execute_trajectory(abb11, abb12, steps["approach_to_AT"], SPEED_APPROACH_AT)
    sync_gantry_off(abb11, abb12)

    # ── 5. Place at assembly target ───────────────────────────────────────────────────────
    print("\n[5/7] place_at_AT")
    # sync_gantry_on(abb11, abb12)
    # execute_trajectory(abb11, abb12, steps["place_at_AT"], SPEED_APPROACH_AT)
    # sync_gantry_off(abb11, abb12)

    place_approach_frame = record.get("place_approach_frame")
    if place_approach_frame is not None:
        print("  Corrective move to place approach frame...")
        abb12.send_and_wait(
            rrc.MoveToFrame(place_approach_frame, SPEED_HOLD, rrc.Zone.FINE,
                            motion_type=rrc.Motion.LINEAR),
            timeout=400.0)
    else:
        print("  WARNING: no place_approach_frame in record, skipping corrective move.")

    # Final Cartesian placement: independent MoveL on R12 (sync OFF)
    print("  Final placement MoveL (sync OFF)...")
    abb12.send_and_wait(
        rrc.MoveToFrame(place_frame, SPEED_PLACE, rrc.Zone.FINE,
                        motion_type=rrc.Motion.LINEAR),
        timeout=400.0)
    # abb12.send(rrc.PrintText("Stopped after place — press Play to continue"))
    abb12.send_and_wait(rrc.Stop(), timeout=400.0)

    # ── 6. Retract from assembly target ────────────────────────────────────────────────────
    print("\n[6/7] retract_from_AT")
    abb12.send_and_wait(rrc.SetTool(TOOL), timeout=5.0)
    place_retract_frame = record.get("place_retract_frame")
    if place_retract_frame is not None:
        print("  MoveL to place retract frame")
        abb12.send_and_wait(
            rrc.MoveToFrame(place_retract_frame, SPEED_PLACE, rrc.Zone.FINE,
                            motion_type=rrc.Motion.LINEAR),
            timeout=400.0)
    else:
        print("  Fallback: joint trajectory retract from AT")
        sync_gantry_on(abb11, abb12)
        execute_trajectory(abb11, abb12, steps["retract_from_AT"], SPEED_HOLD)
        sync_gantry_off(abb11, abb12)

    # ── 7. Return to safe ────────────────────────────────────────────────────────────────────────
    print("\n[7/7] return_to_safe")
    sync_gantry_on(abb11, abb12)
    execute_trajectory(abb11, abb12, steps["return_to_safe"], SPEED_FREE)
    sync_gantry_off(abb11, abb12)

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
        import traceback
        traceback.print_exc()       

    finally:
        disconnect_ros(ros)


if __name__ == "__main__":
    main()
