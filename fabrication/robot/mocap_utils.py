import sys

_ROCAP_PATH = r"C:\Users\paulj\Documents\github\rocap"
if _ROCAP_PATH not in sys.path:
    sys.path.insert(0, _ROCAP_PATH)

from robot_workflow import _load_calib, collect_frames, average_frames, compute_robot_frame  # noqa: E402

MOCAP_WS_URL = "ws://192.168.55.107:8765"  # rocap mocap WebSocket URL (IP of the machine running rocap)
MOCAP_COLLECT_DURATION = 0.5  # seconds to average


def fetch_pickup_frame():
    """Fetch the current pickup frame via the mocap WebSocket (rocap pipeline).

    Returns
    -------
    :class:`compas.geometry.Frame`
        The computed robot-space pickup frame.

    Raises
    ------
    RuntimeError
        If the calibration file is missing or no frames are received.
    """
    try:
        T_calib, T_ftm = _load_calib()
    except FileNotFoundError as e:
        raise RuntimeError("Calibration file not found. Run solve_handeye.py first.") from e

    print("Fetching mocap frames from {} ({:.2f}s)...".format(MOCAP_WS_URL, MOCAP_COLLECT_DURATION))
    frames, errs = collect_frames(MOCAP_WS_URL, MOCAP_COLLECT_DURATION)

    if not frames:
        raise RuntimeError("No mocap frames received from {}. Errors: {}".format(MOCAP_WS_URL, errs))

    pickup_frame, _, _ = compute_robot_frame(average_frames(frames), T_calib, T_ftm)
    print(
        "Pickup frame fetched ({} samples): ({:.1f}, {:.1f}, {:.1f}) mm".format(
            len(frames), pickup_frame.point.x, pickup_frame.point.y, pickup_frame.point.z
        )
    )
    if errs:
        print("WebSocket warnings: {}".format(errs[:3]))
    return pickup_frame
