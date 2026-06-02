import os
import traceback
import threading
import math

import paho.mqtt.client as mqtt
from compas.colors import Color
from compas.data import json_dump
from compas.data import json_load
from compas_fab.viewer import TrajectoryPlayer
from compas_threejs.materials import PhysicalMaterial
from compas_threejs.ui import Button
from compas_threejs.ui import TextLabel
from compas_threejs.viewer import CameraView

from core.utils import combine_trajectories
from mocap_utils import fetch_pickup_frame
from timber.timber_planner import TimberProcessPlanner

STEP_NAMES = [
    "approach_to_pick",
    "pick",
    "retract_from_pick",
    "approach_to_AT",
    "place_at_AT",
    "retract_from_AT",
    "return_to_safe",
]

MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
MQTT_TOPIC = "qr/timba/scan"
QR_SEQ_OFFSET = 1  # QR labels are 1-indexed (m01-01 -> seq_i=0)

STATE_FILE = "fabrication\\data\\fabrication_state.json"
EXPORT_PATH = "fabrication\\data\\fabrication_sequence.json"

# --- Label colours ---
_COL_WAITING  = "#ef4444"   # red   — waiting for QR / mocap
_COL_FETCHED  = "#3b82f6"   # blue  — mocap frame ready
_COL_COMPUTED = "#18181b"   # black — trajectory computed


def _load_state():
    if os.path.exists(STATE_FILE):
        try:
            return json_load(STATE_FILE)
        except Exception:
            pass
    return {"last_assembled": -1}


def _save_state(last_assembled):
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
    json_dump({"last_assembled": last_assembled}, STATE_FILE)


def _beam_label(seq_i, beam_id, suffix, assembly_method):
    """e.g. 'Beam 1/12 — scan QR'"""
    return "Seq Index: {}\n{}\n{}\nPlaced by: {}".format(seq_i, beam_id, suffix, assembly_method)


def main():
    # ---------------------------------------------------------
    # 1. LOAD DATA
    # ---------------------------------------------------------
    print("Loading models...")
    filepath_model = "fabrication\\data\\timber_models\\260529_v1_fabrication_model_1.json"

    if not os.path.exists(filepath_model):
        raise FileNotFoundError("Could not find the model or nesting JSON files. Check your paths.")

    timber_model = json_load(filepath_model)[0].get("model")
    timber_model.process_joinery()

    # ---------------------------------------------------------
    # 2. INITIALIZE PLANNER
    # ---------------------------------------------------------
    print("Initializing Timber Process Planner...")
    trajectory_planner = TimberProcessPlanner(group="robot12_eaXYZ")
    trajectory_planner.setup_physical_cell()

    # ---------------------------------------------------------
    # 3. SEQUENCE — resume from last assembled
    # ---------------------------------------------------------
    in_seq_beams = sorted(
        (obj for obj in timber_model.beams if "sequence_id" in obj.attributes),
        key=lambda x: x.attributes["sequence_id"]
    )
    total_beams = len(in_seq_beams)
    # robot_beams = [b for b in in_seq_beams if b.attributes.get("robot")]

    state = _load_state()
    last_assembled = state.get("last_assembled", -1)
    trajectory_planner.seq_i = last_assembled + 1

    if trajectory_planner.seq_i >= total_beams:
        print("All {} beams already assembled. Nothing to do.".format(total_beams))
        return

    print("Resuming from beam {}/{} (last assembled: {})".format(
        trajectory_planner.seq_i + 1, total_beams, last_assembled))

    # ---------------------------------------------------------
    # 4. LAUNCH VIEWER
    # ---------------------------------------------------------
    print("\nLaunching Trajectory Viewer...")
    player = TrajectoryPlayer(
        robot_cell=trajectory_planner.robot_cell,
        cell_state=trajectory_planner.state,
    )

    last_sequence = {"record": None}
    highlight_state = {"mesh": None}

    # UI: text label tracks the current step state
    id_label = TextLabel(
        text=_beam_label(trajectory_planner.seq_i, in_seq_beams[trajectory_planner.seq_i].name, "scan QR", in_seq_beams[trajectory_planner.seq_i].attributes.get("assembly_method")),
        label="Current Beam",
    )

    def _set_label(suffix, color):
        player.viewer.update_text_label(
            id_label,
            _beam_label(trajectory_planner.seq_i, in_seq_beams[trajectory_planner.seq_i].name, suffix, in_seq_beams[trajectory_planner.seq_i].attributes.get("assembly_method")),
            color=color,
        )

    def _on_qr_received(payload):
        """Called from the MQTT thread when a QR scan is published."""
        if len(player.buttons_to_del) > 0:
            for button in player.buttons_to_del:
                player.viewer.remove_ui_element(button)

        try:
            beam_id = int(payload.split("_")[-1])
        except (ValueError, IndexError):
            print("QR: unrecognised payload '{}'".format(payload))
            return

        if beam_id < 0 or beam_id >= total_beams:
            print("QR: beam_id {} out of range (0-{})".format(beam_id, total_beams - 1))
            _set_label("out of range", _COL_WAITING)
            return

        # Update to the scanned beam (allows jumping forward if needed)
        beam = next((beam for beam in timber_model.beams if int(beam.name.split("_")[-1]) == beam_id), None)
        trajectory_planner.seq_i = beam.attributes.get("sequence_id", None)
        print("QR: {} -> seq_i={} ({})".format(payload, trajectory_planner.seq_i, beam))

        # --- Highlight: remove previous beam, add new one ---
        if highlight_state["mesh"] is not None:
            try:
                player.viewer.remove_object(highlight_state["mesh"])
            except Exception:
                pass

        # --- Fetch pickup frame (blocking, runs in MQTT thread) ---
        _set_label("fetching…", _COL_WAITING)
        try:
            trajectory_planner._fetched_pickup_frame = fetch_pickup_frame()
            _set_label("ready ✓", _COL_FETCHED)
            print("QR: pickup frame ready for seq_i={}. Press Compute.".format(trajectory_planner.seq_i))
        except RuntimeError as e:
            from compas.geometry import Frame, Point, Vector
            trajectory_planner._fetched_pickup_frame = Frame(point=Point(x=16040, y=5076, z=1009), xaxis=Vector(x=-1.000, y=-0.000, z=-0.000), yaxis=Vector(x=0.000, y=1.000, z=0.000)).rotated(math.radians(90), Vector(0,0,1), Point(x=16040, y=7076, z=1009))
            _set_label("retry — fetch failed", _COL_WAITING)
            print("QR: fetch FAILED - {}".format(e))
        
        buttons_to_del = []
        if beam.attributes.get("assembly_method") == "robot":
            compute_button = Button(text="Compute Trajectories", action=_on_compute, label=None)
            buttons_to_del.append(compute_button)
            export_button = Button(text="Export Trajectory", action=_on_export, label=None)
            buttons_to_del.append(export_button)
            player.viewer.add_ui_element(compute_button)
            player.viewer.add_ui_element(export_button)
        confirm_button = Button(text="Confirm Assembly", action=_on_confirm, label=None)
        buttons_to_del.append(confirm_button)
        player.viewer.add_ui_element(confirm_button)
        player.buttons_to_del = buttons_to_del

        player._cleanup_previous_run()

        # clear the QR highlight
        if highlight_state["mesh"] is not None:
            try:
                player.viewer.remove_object(highlight_state["mesh"])
            except Exception:
                pass
            highlight_state["mesh"] = None

        trajectory_planner.state.robot_configuration = trajectory_planner.safe_configuration
        if os.path.exists(EXPORT_PATH):
            record = json_load(EXPORT_PATH)
            trajectory_planner.update_state_from_trajectory(record["steps"]["return_to_safe"])
        
        rb_names = trajectory_planner.robot_cell.rigid_body_models.keys()
        for rb_name in list(rb_names):
            if rb_name != "t2_rfl_colmesh":
                trajectory_planner.robot_cell.rigid_body_models.pop(rb_name)
                trajectory_planner.state.rigid_body_states.pop(rb_name)

        assembled_elements = []
        assembled_elements.clear() 
        for p in timber_model.plates[:1]:
            p_mesh = p.geometry.to_viewmesh()[0]
            assembled_elements.append(p_mesh)
        for b in in_seq_beams[:trajectory_planner.seq_i]:
            b_mesh = b.geometry.to_viewmesh()[0]
            assembled_elements.append(b_mesh)

        trajectory_planner.add_rb_to_cell(meshes=assembled_elements, name="assembled_elements")
        player._draw_assembled_elements(assembled_elements)
        
        if hasattr(trajectory_planner, 'workpiece_manager'):
            wm = trajectory_planner.workpiece_manager
            wm.rules.clear()
            wm.meshes.clear()
            wm.latest_stock_vanish_time = 0.0
            wm.lumber_yard_stock_y = 0.0

        if hasattr(trajectory_planner, 'trajectory_list'):
            trajectory_planner.trajectory_list = []

        if hasattr(trajectory_planner, 'current_time'):
            trajectory_planner.current_time = 0.0

        if hasattr(trajectory_planner, 'planned_time'):
            trajectory_planner.planned_time = 0.0

        if trajectory_planner._fetched_pickup_frame is None:
            print("ERROR: Fetch pickup frame first before computing.")
            return

        try:
            highlight_mesh = beam.geometry.to_viewmesh()[0]
            highlight_mat = PhysicalMaterial(color=Color(1.0, 0.55, 0.0), roughness=0.4, opacity=0.9)
            player.viewer.add_geometry(highlight_mesh, highlight_mat)
            highlight_state["mesh"] = highlight_mesh
        except Exception as e:
            print("QR: highlight failed - {}".format(e))
            traceback.print_exc()


    def _mqtt_setup():
        def _on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("MQTT: connected to {}:{} topic={}".format(MQTT_BROKER, MQTT_PORT, MQTT_TOPIC))
                client.subscribe(MQTT_TOPIC, qos=1)
            else:
                print("MQTT: connect failed rc={}".format(rc))

        def _on_message(client, userdata, msg):
            payload = msg.payload.decode("utf-8").strip()
            print("MQTT: received '{}'".format(payload))
            _on_qr_received(payload)

        def _on_disconnect(client, userdata, rc):
            print("MQTT: disconnected (rc={}), will auto-reconnect".format(rc))

        client = mqtt.Client()
        client.on_connect = _on_connect
        client.on_message = _on_message
        client.on_disconnect = _on_disconnect
        client.reconnect_delay_set(min_delay=2, max_delay=30)
        try:
            client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
            client.loop_forever()
        except Exception as e:
            print("MQTT: failed to start - {}".format(e))

    mqtt_thread = threading.Thread(target=_mqtt_setup, name="mqtt-qr", daemon=True)
    mqtt_thread.start()
    print("MQTT: QR listener started ({}/{})".format(MQTT_BROKER, MQTT_TOPIC))

    # --- Button: Compute Trajectories ---
    def _on_compute():
        # if os.path.exists(EXPORT_PATH):
        #     record = json_load(EXPORT_PATH)
        #     trajectory_planner.update_state_from_trajectory(record["steps"]["return_to_safe"])
        beam = in_seq_beams[trajectory_planner.seq_i]
        next_robotb = next((b for b in in_seq_beams[trajectory_planner.seq_i + 1:] if b.attributes.get("assembly_method") == "robot"), None)
        next_robotb_location = next_robotb.attributes.get("grasp_frame").point if next_robotb else None
        beam.attributes["next_robotb_location"] = next_robotb_location
        beam.attributes["plate_outline"] = timber_model.plates[0].outline_a

        player._cleanup_previous_compute()

        if hasattr(trajectory_planner, 'workpiece_manager'):
            wm = trajectory_planner.workpiece_manager
            wm.rules.clear()
            wm.meshes.clear()
            wm.latest_stock_vanish_time = 0.0
            wm.lumber_yard_stock_y = 0.0

        if hasattr(trajectory_planner, 'trajectory_list'):
            trajectory_planner.trajectory_list = []

        if hasattr(trajectory_planner, 'current_time'):
            trajectory_planner.current_time = 0.0

        if hasattr(trajectory_planner, 'planned_time'):
            trajectory_planner.planned_time = 0.0

        if trajectory_planner._fetched_pickup_frame is None:
            print("ERROR: Fetch pickup frame first before computing.")
            return

        print("\n{}".format("X" * 40))
        print("PLANNING element {} of {}: {}".format(
            beam.attributes.get("sequence_id"), total_beams, beam.name))
        print("{}".format("X" * 40))
        try:
            element_trajectories = trajectory_planner.pick_and_place_element(
                str(beam.guid), timber_model
            )
        except Exception as e:
            print("ERROR during planning: {}".format(e))
            traceback.print_exc()
            return

        failed = [i for i, t in enumerate(element_trajectories) if t is None]
        valid = [t for t in element_trajectories if t is not None]

        print("\nCombining trajectories...")
        if not valid:
            print("ERROR: all {} step(s) failed. Cannot combine.".format(len(failed)))
            return
        if failed:
            print("WARNING: step(s) {} failed, combining {} of {} steps.".format(
                failed, len(valid), len(element_trajectories)))
            last_sequence["record"] = None
        else:
            _approach = getattr(trajectory_planner, "_last_approach_frame", None)
            _place = getattr(trajectory_planner, "_last_place_frame", None)
            _pick_retract = getattr(trajectory_planner, "_last_pick_retract_frame", None)
            _place_retract = getattr(trajectory_planner, "_last_place_retract_frame", None)
            last_sequence["record"] = {
                "index": trajectory_planner.seq_i,
                "beam_guid": str(beam.guid),
                "pickup_frame": trajectory_planner._fetched_pickup_frame,
                "approach_frame": _approach.scaled(1000) if _approach is not None else None,
                "place_frame": _place.scaled(1000) if _place is not None else None,
                "pick_retract_frame": _pick_retract.scaled(1000) if _pick_retract is not None else None,
                "place_retract_frame": _place_retract.scaled(1000) if _place_retract is not None else None,
                "steps": {
                    name: element_trajectories[i] if i < len(element_trajectories) else None
                    for i, name in enumerate(STEP_NAMES)
                },
            }

        merged_trajectory = combine_trajectories(valid)

        player.trajectory = merged_trajectory
        player.add_dynamic_workpieces(
            pnp_data=trajectory_planner.workpiece_manager.rules,
            geometry_dict=trajectory_planner.workpiece_manager.meshes,
        )
        player.add_visual_helpers(trace=True, triad=True, ghost=False, group=trajectory_planner.group)
        player._setup_scrubber()
        if hasattr(player, "_scrub_callback"):
            player._scrub_callback([0])

        _set_label("computed ✓", _COL_COMPUTED)
        print("Done. Use the timeline to preview the trajectory.")

    # --- Button: Export Trajectory ---
    def _on_export():
        if last_sequence["record"] is None:
            print("ERROR: No fully computed sequence to export. Compute all steps successfully first.")
            return
        json_dump(last_sequence["record"], EXPORT_PATH)
        exported_seq_i = last_sequence["record"]["index"]
        print("Exported sequence {} to: {}".format(exported_seq_i, EXPORT_PATH))

    def _on_confirm():
        confirmed_seq_i = trajectory_planner.seq_i
        _save_state(confirmed_seq_i)
        next_seq_i = confirmed_seq_i + 1
        if next_seq_i < total_beams:
            trajectory_planner.seq_i = next_seq_i
            trajectory_planner._fetched_pickup_frame = None
            last_sequence["record"] = None
            _set_label("scan QR", _COL_WAITING)
            print("Ready for beam {}/{}.".format(next_seq_i + 1, total_beams))
        else:
            _set_label("all done!", _COL_COMPUTED)
            print("All {} beams assembled!".format(total_beams))
        
        for button in player.buttons_to_del:
            player.viewer.remove_ui_element(button)



    # --- Button: Prev / Next beam ---
    def _jump_to(seq_i):
        trajectory_planner.seq_i = seq_i
        trajectory_planner._fetched_pickup_frame = None
        last_sequence["record"] = None
        if highlight_state["mesh"] is not None:
            try:
                player.viewer.remove_object(highlight_state["mesh"])
            except Exception:
                pass
            highlight_state["mesh"] = None
        _set_label("scan QR", _COL_WAITING)
        print("Jumped to beam {}/{}.".format(seq_i + 1, total_beams))

    def _on_prev():
        new_i = trajectory_planner.seq_i - 1
        if new_i < 0:
            print("Already at first beam.")
            return
        _jump_to(new_i)

    def _on_next():
        new_i = trajectory_planner.seq_i + 1
        if new_i >= total_beams:
            print("Already at last beam.")
            return
        _jump_to(new_i)

    player.viewer.picker = True
    player.viewer.set_view(CameraView.FRONT_RIGHT)
    player.viewer.add_ui_element(id_label)
    player.viewer.add_ui_element(Button(text="← Prev", action=_on_prev, label=None))
    player.viewer.add_ui_element(Button(text="Next →", action=_on_next, label=None))

    player.show()


if __name__ == "__main__":
    main()
