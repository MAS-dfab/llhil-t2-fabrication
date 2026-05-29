"""hops_exporter.py
===================
Grasshopper-compatible beam visualiser / .hop exporter.

GH Python component usage
--------------------------
Inputs (Item Access, No Type Hint unless noted):
    filepath    : str  — path to the JSON fabrication model
    index       : int  — beam idx attribute (idx-mode)
    hierarchy   : str  — beam hierarchy, e.g. "shoe", "primary" (type-mode)
    type_index  : int  — 0-based position within the filtered hierarchy list (type-mode)
    export      : bool — write .hop file to <filepath_dir>/hops/<beam.name>.hop

When hierarchy is set, type-mode is used and index is ignored.
When hierarchy is None, idx-mode is used and index is matched against beam.attributes["idx"].

Outputs:
    geometry          — Rhino geometry drawn by the scene
    processing_report — list[str], one entry per BTLx processing on the beam
                        format: "PROCESSING_NAME | ref_side: N"

Note: add 'processing_report' as a named output parameter on the GH Python component.

GH component body (minimal):
    import os, sys, importlib

    gh_dir = os.path.dirname(ghenv.Component.OnPingDocument().FilePath)
    # go up from your GH file to the repo root, then down to fabrication/cnc
    # adjust the number of dirname() calls to match your GH file's folder depth
    repo_root = os.path.dirname(os.path.dirname(gh_dir))  # e.g. design/modularization -> design -> repo root
    cnc_dir = os.path.join(repo_root, "fabrication", "cnc")
    if cnc_dir not in sys.path:
        sys.path.insert(0, cnc_dir)

    import hops_exporter
    importlib.reload(hops_exporter)

    geometry, processing_report = hops_exporter.run(filepath, index, hierarchy, type_index, export, ghenv)
"""

import math
import os

import Grasshopper.Kernel as gh

from compas.data import json_load
from compas.geometry import Rotation
from compas.geometry import Translation
from compas.geometry import Vector
from compas.scene import Scene
from compas_timber.fabrication import JackRafterCut

from easyhops.hop_job import HOPSJob, HOPSMachining, WorkPlane
from easyhops.hop_core import EasySnapXY
from easyhops.hop_core import EasySnapZ
from easyhops.utility_commands import MachineStop
from easyhops.strategies import LapStrategies
from easyhops.strategies import JackRafterCutStrategies
from easyhops.strategies import LongitudinalCutStrategies
from easyhops.strategies import DrillingStrategies
from easyhops.machining_commands import SawYOperation
from easyhops.machining_commands import CompensationMode
from easyhops.tool_library import SaegeD350
from easyhops.tool_library import CastorD61
from easyhops.tool_library import SRSLD12


# ---------------------------------------------------------------------------
# Model loading
# ---------------------------------------------------------------------------


def load_model(filepath):
    model_dict = json_load(filepath)
    model = model_dict[0]["model"]
    model.process_joinery()

    return model


# ---------------------------------------------------------------------------
# Beam resolution
# ---------------------------------------------------------------------------


def override_features(beam):
    """Replace any JackRafterCut features on the beam with new features based on the same planes but with ref_side_index taken from beam attributes."""
    ref_side_index = beam.attributes.get("ref_side_index", 0)
    # if beam.attributes.get("assembly_method") == "robot":
    #     ref_side_index = (
    #         ref_side_index + 2
    #     ) % 4  # flip ref side for robot-assembled beams

    for f in beam.features:
        if isinstance(f, JackRafterCut):
            plane = f.plane_from_params_and_beam(beam)
            new_feature = f.__class__.from_plane_and_beam(plane, beam, ref_side_index)
            beam.remove_features(f)
            beam.add_feature(new_feature)


def resolve_beam(model, index, hierarchy):
    """Return the single beam matching the given selection criteria.

    Type-mode (hierarchy is not None):
        Filters beams by hierarchy, then picks the beam at position type_index.
        Prints: "[hierarchy] current/total | beam: name"

    Idx-mode (hierarchy is None):
        Finds the beam where beam.attributes["idx"] == index.

    Returns the beam, or None if no match (GH warning added in that case).
    """
    warn = gh.GH_RuntimeMessageLevel.Warning

    if hierarchy is not None:
        filtered = [
            b for b in model.beams if b.attributes.get("hierarchy") == hierarchy
        ]
        total = len(filtered)

        if total == 0:
            ghenv.Component.AddRuntimeMessage(
                warn,
                "No beams found with hierarchy '{}'.".format(hierarchy),
            )
            return None

        if index is None or index < 0 or index >= total:
            ghenv.Component.AddRuntimeMessage(
                warn,
                "index {} out of range. [{}] has {} beam(s) (0\u2013{}).".format(
                    index, hierarchy, total, total - 1
                ),
            )
            return None

        beam = filtered[index]
        print("[{}] {}/{} | beam: {}".format(hierarchy, index + 1, total, beam.name))
        return beam

    else:
        for beam in model.beams:
            if beam.attributes.get("idx") == index:
                return beam
        ghenv.Component.AddRuntimeMessage(
            warn,
            "No beam found with idx={}.".format(index),
        )
        return None


# ---------------------------------------------------------------------------
# Visualisation
# ---------------------------------------------------------------------------


def visualize_geometry(beam):
    """Transform the beam to the origin and add blank edges + featured geometry to scene."""
    scene = Scene()

    ref_side_index = beam.attributes.get("ref_side_index", 0)
    # if ref_side_index is None:
    #     raise ValueError(
    #         "Beam {} is missing 'ref_side_index' attribute.".format(beam.name)
    #     )
    width, _ = beam.get_dimensions_relative_to_side(ref_side_index)

    ref_frame = beam.ref_frame.transformed(beam.transformation_to_local())
    translation = Translation.from_vector(
        Vector.from_start_end(ref_frame.point, [0, 0, 0])
    ) * Translation.from_vector([0, width, 0])
    rotation = Rotation.from_axis_and_angle([1, 0, 0], math.pi * ref_side_index / 2)
    transformation = translation * rotation

    # blank edges
    blank_brp = beam.compute_elementgeometry(include_features=False)
    blank_brp.transform(transformation)
    for edge in blank_brp.edges:
        scene.add(edge.curve)

    # geometry with features
    geometry = beam.compute_elementgeometry(include_features=True)
    scene.add(geometry.transformed(transformation))
    return scene.draw()


# ---------------------------------------------------------------------------
# Processing report
# ---------------------------------------------------------------------------


def get_processing_report(beam):
    """Return a list of strings describing each BTLx processing on the beam.

    Each entry has the format: "PROCESSING_NAME | ref_side: N"
    """
    report = [
        "Beam: {} | RSId: {}".format(beam.name, beam.attributes.get("ref_side_index"))
    ]
    report.append("-------------")
    report.extend(
        "{} | RSId: {}".format(feature.PROCESSING_NAME, feature.ref_side_index)
        for feature in beam.features
    )
    return report


# ---------------------------------------------------------------------------
# Export HOPS
# ---------------------------------------------------------------------------


def _element_to_job(element, scale_factor=1000.0):
    """Build a HOPSJob for *element* with explicit control over dispatch."""
    job = HOPSJob.from_element(element, scale_factor=scale_factor)
    rsi = job.ref_side_index
    opp_rsi = (rsi + 2) % 4

    pre_flip = []
    post_flip = []

    # Add cuts at the end of the part at the pre-flip stage.
    cut_end = SawYOperation(
        radius_compensation=CompensationMode.RIGHT,
        easy_snap_xy=EasySnapXY.FRONT_RIGHT,
        easy_snap_z=EasySnapZ.BOTTOM_SIDE,
    )

    pre_flip.append(
        HOPSMachining(
            tool=SaegeD350(),
            work_plane=WorkPlane.TOP,
            operations=[cut_end],
            comments=[
                "; ---------------------------------",
                ";ENDCut_Sawing",
                "; ---------------------------------",
            ],
        )
    )

    for processing in element.features:
        processing = processing.scaled(scale_factor)
        name = processing.PROCESSING_NAME

        if name == "LongitudinalCut":
            machinings = LongitudinalCutStrategies.contouring(
                processing, machine_ref_side_index=rsi, tool=CastorD61()
            )
            post_flip.extend(machinings)

        elif name == "Lap":
            machinings = LapStrategies.milling(
                processing, machine_ref_side_index=rsi, tool=CastorD61()
            )
            if processing.ref_side_index == rsi:
                pre_flip.extend(machinings)
            elif processing.ref_side_index == opp_rsi:
                post_flip.extend(machinings)
            else:
                raise ValueError(
                    f"Unexpected ref_side_index {processing.ref_side_index} for Lap"
                )

        elif name == "JackRafterCut":
            assert processing.ref_side_index in (rsi, opp_rsi), (
                f"Unexpected ref_side_index {processing.ref_side_index} for JackRafterCut"
            )
            post_flip.extend(
                JackRafterCutStrategies.sawing(
                    processing, machine_ref_side_index=rsi, tool=SaegeD350()
                )
            )

        elif name == "Drilling":
            machinings = DrillingStrategies.pocketing(
                processing, machine_ref_side_index=rsi, tool=SRSLD12()
            )
            if processing.ref_side_index == rsi:
                pre_flip.extend(machinings)
            elif processing.ref_side_index == opp_rsi:
                post_flip.extend(machinings)
            else:
                raise ValueError(
                    f"Unexpected ref_side_index {processing.ref_side_index} for Drilling"
                )

    # Sort pre-flip: milling before sawing
    pre_flip.sort(
        key=lambda m: {"SAWING": 0, "DRILLING": 1, "MILLING": 2}.get(
            getattr(m, "OPERATION_TYPE", ""), 3
        )
    )

    # Sort post-flip: milling before sawing
    post_flip.sort(
        key=lambda m: {"MILLING": 0, "SAWING": 1}.get(
            getattr(m, "OPERATION_TYPE", ""), 2
        )
    )

    if pre_flip:
        job.add(pre_flip)
        job.add(MachineStop("flip beam 180deg"))
    job.add(post_flip)

    return job


def export_hop(beam, export_dir):
    """Write a .hop file for beam into export_dir/<beam.name>.hop."""
    os.makedirs(export_dir, exist_ok=True)
    hop_path = os.path.join(export_dir, beam.name + ".hop")
    job = _element_to_job(beam)
    job.to_hop_file(hop_path)
    return job


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------


def run(filepath, index, hierarchy, export):
    """Resolve one beam, visualise it, optionally export it, and return a processing report.

    Returns:
        geometry          — result of scene.draw()
        processing_report — list[str] of processing descriptions
    """
    model = load_model(filepath)
    export_dir = os.path.join(os.path.dirname(filepath), "hops")

    beam = resolve_beam(model, index, hierarchy)
    if beam is None:
        return None, []

    geometry = visualize_geometry(beam)
    export_hop(beam, export_dir, export)
    processing_report = get_processing_report(beam)

    return geometry, processing_report
