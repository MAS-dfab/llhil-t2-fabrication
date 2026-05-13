import os

from compas.data import json_dump
from compas.data import json_load
from compas_fab.viewer import TrajectoryPlayer
from compas_threejs.ui import Button

from core.utils import combine_trajectories
from mocap_utils import fetch_pickup_frame
from timber.timber_planner import TimberProcessPlanner


def main():
    # ---------------------------------------------------------
    # 1. LOAD DATA
    # ---------------------------------------------------------
    print("Loading models...")
    filepath_model = "fabrication\\data\\models\\t2_tm_0.json"

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
    seq_i = 0
    assembled_elements = []

    # for p in timber_model.plates:
    #     p_mesh = p.elementgeometry.transformed(trajectory_planner.at_T).to_viewmesh()[0]
    #     assembled_elements.append(p_mesh)

    in_seq_beams = sorted(
        (obj for obj in timber_model.beams if "sequence" in obj.attributes),
        key=lambda x: x.attributes["sequence"]
    )

    for b in in_seq_beams[:seq_i]:
        b_mesh = b.geometry.transformed(trajectory_planner.at_T * b.attributes.get("parent_T")).to_viewmesh()[0]
        assembled_elements.append(b_mesh)
    trajectory_planner.add_rb_to_cell(meshes=assembled_elements, name="assembled_elements")

    beam = in_seq_beams[seq_i]

    # ---------------------------------------------------------
    # 4. LAUNCH VIEWER (no trajectory yet)
    # ---------------------------------------------------------
    print("\nLaunching Trajectory Viewer...")
    player = TrajectoryPlayer(
        robot_cell=trajectory_planner.robot_cell,
        cell_state=trajectory_planner.state,
    )


    export_path = "fabrication\\data\\merged_trajectory.json"

    # --- Button: Fetch Pickup Frame ---
    def _on_fetch():
        try:
            trajectory_planner._fetched_pickup_frame = fetch_pickup_frame()
            print("Pickup frame ready. Press Compute.")
        except RuntimeError as e:
            print("ERROR fetching pickup frame: {}".format(e))

    # --- Button: Compute Trajectories ---
    def _on_compute():
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
        if player.trajectory is None:
            print("ERROR: No trajectory to export. Compute first.")
            return
        json_dump(player.trajectory, export_path)
        print("Trajectory exported to: {}".format(export_path))

    player.viewer.add_ui_element(Button(text="Fetch Pickup Frame", action=_on_fetch, label="Fetch"))
    player.viewer.add_ui_element(Button(text="Compute Trajectories", action=_on_compute, label="Compute"))
    player.viewer.add_ui_element(Button(text="Export Trajectory", action=_on_export, label="Export"))

    player.show()


if __name__ == "__main__":
    main()
