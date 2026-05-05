import os

from compas.data import json_dump
from compas.data import json_load
from compas_fab.viewer import TrajectoryPlayer

from core.utils import combine_trajectories
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
    # 3. EXECUTE PLANNING LOOP
    # ---------------------------------------------------------
    seq_i = 5

    trajectories = []
    assembled_elements = []

    for p in timber_model.plates:
        p_mesh = p.elementgeometry.transformed(trajectory_planner.at_T).to_viewmesh()[0]
        assembled_elements.append(p_mesh)

    # Sort beams by sequence attribute to ensure correct assembly order
    in_seq_beams = sorted(
    (obj for obj in timber_model.beams if "sequence" in obj.attributes), 
    key=lambda x: x.attributes["sequence"]
    )

    # Add already assembled beams to the cell for collision checking
    for b in in_seq_beams[:seq_i]:
        b_mesh = b.geometry.transformed(trajectory_planner.at_T*b.attributes.get("parent_T")).to_viewmesh()[0]
        assembled_elements.append(b_mesh)
    trajectory_planner.add_rb_to_cell(meshes=assembled_elements, name="assembled_elements")

    # Compute trajectory for current beam
    beam = in_seq_beams[seq_i]
    print(f"\n{'X'*40}")
    print(f"PLANNING element {beam.attributes.get('sequence')} of {len(in_seq_beams)}: {beam}")
    print(f"{'X'*40}")
    
    element_trajectories = trajectory_planner.pick_and_place_element(
        str(beam.guid),
        timber_model
    )
    trajectories.extend(element_trajectories)

    # ---------------------------------------------------------
    # 4. POST-PROCESSING & EXPORT
    # ---------------------------------------------------------
    print("\nCombining trajectories...")
    # merged_trajectory = combine_trajectories(trajectory_planner.trajectory_list)
    merged_trajectory = combine_trajectories(element_trajectories)
    
    export_path = "C:\\Users\\paulj\\Downloads\\merged_traj_test.json"
    merged_trajectory.to_json(export_path)
    print(f"Successfully exported merged trajectory to: {export_path}")

    # ---------------------------------------------------------
    # 5. LAUNCH VIEWER
    # ---------------------------------------------------------

    print("\n🚀 Launching Trajectory Viewer...")
    player = TrajectoryPlayer(
        robot_cell=trajectory_planner.robot_cell, 
        trajectory=merged_trajectory,
        cell_state=trajectory_planner.state,
        # use_cache=True
    )
    
    player.add_dynamic_workpieces(
        pnp_data=trajectory_planner.workpiece_manager.rules, 
        geometry_dict=trajectory_planner.workpiece_manager.meshes
    )
    
    player.add_visual_helpers(
        trace=True, 
        triad=True, 
        ghost=False,
        group=trajectory_planner.group
    )
        
    player.show()

if __name__ == "__main__":
    main()