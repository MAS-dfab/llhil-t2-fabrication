import os
import threading

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

_BG_ERROR = Color(0.8, 0.1, 0.1)       # red: fetch failed


def main():
    # ---------------------------------------------------------
    # 1. LOAD DATA
    # ---------------------------------------------------------
    print("Loading models...")
    filepath_model = "fabrication\\data\\models\\tm_test.json"

    if not os.path.exists(filepath_model):
        raise FileNotFoundError("Could not find the model or nesting JSON files. Check your paths.")

    timber_model = json_load(filepath_model)
    timber_model.process_joinery()

    # ---------------------------------------------------------
    # 2. INITIALIZE PLANNER
    # ---------------------------------------------------------
    print("Initializing Timber Process Planner...")
    trajectory_planner = TimberProcessPlanner(group="robot12_eaXYZ")
    trajectory_planner.setup_physical_cell()

    # ---------------------------------------------------------
    # 3. PREPARE SCENE (collision objects only — no trajectory yet)
    # ---------------------------------------------------------
    trajectory_planner.seq_i = 0
    # assembled_elements = []

    

    in_seq_beams = sorted(
        (obj for obj in timber_model.beams if "sequence" in obj.attributes),
        key=lambda x: x.attributes["sequence"]
    )



    # for p in timber_model.plates:
    #     p_mesh = p.elementgeometry.transformed(trajectory_planner.at_T).to_viewmesh()[0]
    #     assembled_elements.append(p_mesh)
    # for b in in_seq_beams[:seq_i]:
    #     b_mesh = b.geometry.transformed(trajectory_planner.at_T * b.attributes.get("parent_T")).to_viewmesh()[0]
    #     assembled_elements.append(b_mesh)
    # trajectory_planner.add_rb_to_cell(meshes=assembled_elements, name="assembled_elements")

    

    # ---------------------------------------------------------
    # 4. LAUNCH VIEWER (no trajectory yet)
    # ---------------------------------------------------------
    print("\nLaunching Trajectory Viewer...")
    player = TrajectoryPlayer(
        robot_cell=trajectory_planner.robot_cell,
        cell_state=trajectory_planner.state,
    )


    export_path = "fabrication\\data\\fabrication_sequence.json"
    last_sequence = {"record": None}  # mutable container so closure can write to it

    # --- QR / MQTT state ---
    highlight_state = {"mesh": None}  # tracks the currently highlighted beam mesh

    # UI text label — shows current QR id in the sidebar panel
    id_label = TextLabel(text="-", label="Current Beam")

    def _on_qr_received(payload):
        """Called from the MQTT thread when a QR scan is published."""
        try:
            seq_i = int(payload.split("-")[-1]) - QR_SEQ_OFFSET
        except (ValueError, IndexError):
            print("QR: unrecognised payload '{}'".format(payload))
            return

        if seq_i < 0 or seq_i >= len(in_seq_beams):
            print("QR: seq_i {} out of range (0-{})".format(seq_i, len(in_seq_beams) - 1))
            player.viewer.background_color = _BG_ERROR
            return

        trajectory_planner.seq_i = seq_i
        beam = in_seq_beams[seq_i]
        print("QR: {} -> seq_i={} ({})".format(payload, seq_i, beam))
        player.viewer.update_text_label(id_label, payload)

        # --- Highlight: remove previous beam, add new one ---
        if highlight_state["mesh"] is not None:
            try:
                player.viewer.remove_object(highlight_state["mesh"])
            except Exception:
                pass

        try:
            T_place = trajectory_planner.at_T * beam.attributes.get("parent_T")
            highlight_mesh = beam.geometry.transformed(T_place).to_viewmesh()[0]
            highlight_mat = PhysicalMaterial(color=Color(1.0, 0.55, 0.0), roughness=0.4, opacity=0.9)
            player.viewer.add_geometry(highlight_mesh, highlight_mat)
            highlight_state["mesh"] = highlight_mesh
        except Exception as e:
            print("QR: highlight failed - {}".format(e))

        # --- Fetch pickup frame (blocking, runs in MQTT thread) ---
        try:
            trajectory_planner._fetched_pickup_frame = fetch_pickup_frame()
            print("QR: pickup frame ready for seq_i={}. Press Compute.".format(seq_i))
        except RuntimeError as e:
            player.viewer.background_color = _BG_ERROR
            trajectory_planner._fetched_pickup_frame = None
            print("QR: fetch FAILED - {}".format(e))

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

        # trajectory_planner.seq_i +=1
        beam = in_seq_beams[trajectory_planner.seq_i]
        player._cleanup_previous_run()

        # clear the QR highlight now that we're computing
        if highlight_state["mesh"] is not None:
            try:
                player.viewer.remove_object(highlight_state["mesh"])
            except Exception:
                pass
            highlight_state["mesh"] = None

        trajectory_planner.state.robot_configuration = trajectory_planner.safe_configuration

        assembled_elements = []
        assembled_elements.clear() 
        for p in timber_model.plates[:1]:
            parent_T = p.transformation_to_local()
            p_mesh = p.elementgeometry.transformed(trajectory_planner.at_T).to_viewmesh()[0]
            assembled_elements.append(p_mesh)
        beam.attributes["parent_T"] = parent_T  
        for b in in_seq_beams[:trajectory_planner.seq_i]:
            b.attributes["parent_T"] = parent_T
            b_mesh = b.geometry.transformed(trajectory_planner.at_T * b.attributes.get("parent_T")).to_viewmesh()[0]
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
        print("\n{}".format("X" * 40))
        print("PLANNING element {} of {}: {}".format(
            beam.attributes.get("sequence"), len(in_seq_beams), beam))
        print("{}".format("X" * 40))
        try:
            element_trajectories = trajectory_planner.pick_and_place_element(
                str(beam.guid), timber_model
            )
        except Exception as e:
            print("ERROR during planning: {}".format(e))
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
            # partial result — do not store for export
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

        # Wire trajectory into the player and activate playback
        player.trajectory = merged_trajectory
        player.add_dynamic_workpieces(
            pnp_data=trajectory_planner.workpiece_manager.rules,
            geometry_dict=trajectory_planner.workpiece_manager.meshes,
        )
        player.add_visual_helpers(trace=True, triad=True, ghost=False, group=trajectory_planner.group)
        player._setup_scrubber()
        if hasattr(player, "_scrub_callback"):
            player._scrub_callback([0])
        print("Done. Use the timeline to preview the trajectory.")

    # --- Button: Export Trajectory ---
    def _on_export():
        if last_sequence["record"] is None:
            print("ERROR: No fully computed sequence to export. Compute all steps successfully first.")
            return
        json_dump(last_sequence["record"], export_path)
        print("Exported sequence {} to: {}".format(last_sequence["record"]["index"], export_path))

    player.viewer.picker = True
    player.viewer.set_view(CameraView.FRONT_RIGHT)
    player.viewer.add_ui_element(id_label)
    player.viewer.add_ui_element(Button(text="Compute Trajectories", action=_on_compute, label="Compute"))
    player.viewer.add_ui_element(Button(text="Export Trajectory", action=_on_export, label="Export"))

    player.show()


if __name__ == "__main__":
    main()
