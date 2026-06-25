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
from compas.tolerance import TOL
from compas_timber.fabrication import DoubleCut
from compas_timber.fabrication import JackRafterCut
from compas_timber.fabrication import JackRafterCutProxy

from easyhops.hop_job import HOPSJob
from easyhops.hop_core import ParkMode
from easyhops.utility_commands import MachineStop
from easyhops.strategies import LapStrategies
from easyhops.strategies import JackRafterCutStrategies
from easyhops.strategies import LongitudinalCutStrategies
from easyhops.strategies import DrillingStrategies
from easyhops.strategies import DoubleCutStrategies
from easyhops.strategies import BirdsMouthStrategies
from easyhops.strategies import StepJointStrategies
from easyhops.tool_library import SaegeD350
from easyhops.tool_library import CastorD61
from easyhops.tool_library import SRSLD12

TOOL_MAX_DEPTH = 100.0  # mm, maximum depth of cut for the tools used in this exporter

# ---------------------------------------------------------------------------
# Model loading
# ---------------------------------------------------------------------------


def load_model(filepath, module):
    model_dict = json_load(filepath)
    model = model_dict[module]["model"]
    model.process_joinery()

    return model


# ---------------------------------------------------------------------------
# Beam resolution
# ---------------------------------------------------------------------------


def override_features(beam):
    """Ensure all JackRafterCut features are on ref_side or opp_side, and add opp_side cuts where the blade can't cut through in one pass."""
    ref_side_index = beam.attributes.get("ref_side_index", 0)
    opp_side = (ref_side_index + 2) % 4
    _, height = beam.get_dimensions_relative_to_side(ref_side_index)

    for f in list(beam.features):
        if isinstance(f, DoubleCut):
            if f.user_attributes.get("strategy", None) == "pocketing":
                planes = f.planes_from_params_and_beam(beam)
                new_feature = f.__class__.from_planes_and_beam(
                    planes, beam, ref_side_index
                )
                new_feature.user_attributes.update(f.user_attributes)
                beam.remove_features(f)
                beam.add_feature(new_feature)
            continue

        elif isinstance(f, JackRafterCut) or isinstance(f, JackRafterCutProxy):
            plane = f.plane_from_params_and_beam(beam)
            if f.ref_side_index in [ref_side_index, opp_side]:
                inclination_rad = math.radians(f.inclination)
                blade_path_mm = (height * 1000) / math.sin(inclination_rad)

                if blade_path_mm > TOOL_MAX_DEPTH:
                    beam.add_feature(
                        f.__class__.from_plane_and_beam(
                            plane, beam, ref_side_index=(f.ref_side_index + 2) % 4
                        )
                    )
            else:
                raise ValueError(
                    f"Unexpected ref_side_index {f.ref_side_index} for {f.__class__.__name__} on beam {beam.name}"
                )


def resolve_beam(model, index, level, names=None):
    """Return the single beam matching the given selection criteria.

    Type-mode (hierarchy is not None):
        Filters beams by hierarchy, then picks the beam at position type_index.
        Prints: "[hierarchy] current/total | beam: name"

    Idx-mode (hierarchy is None):
        Finds the beam where beam.attributes["idx"] == index.

    Returns the beam, or None if no match (GH warning added in that case).
    """
    warn = gh.GH_RuntimeMessageLevel.Warning
    hierarchy = None
    if names is not None:
        filtered = [b for b in model.beams if b.name in names]
        total = len(filtered)

        if total == 0:
            ghenv.Component.AddRuntimeMessage(
                warn,
                "No beams found matching the provided names list.",
            )
            return None

        if index is None or index < 0 or index >= total:
            ghenv.Component.AddRuntimeMessage(
                warn,
                "index {} out of range. names filter matched {} beam(s) (0\u2013{}).".format(
                    index, total, total - 1
                ),
            )
            return None

        beam = filtered[index]
        print("[names] {}/{} | beam: {}".format(index + 1, total, beam.name))
        return beam

    elif hierarchy is not None:
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

    elif level is not None:
        filtered = [b for b in model.beams if b.attributes.get("level") == level]
        total = len(filtered)

        if total == 0:
            ghenv.Component.AddRuntimeMessage(
                warn,
                "No beams found with level '{}'.".format(level),
            )
            return None

        if index is None or index < 0 or index >= total:
            ghenv.Component.AddRuntimeMessage(
                warn,
                "index {} out of range. [level={}] has {} beam(s) (0\u2013{}).".format(
                    index, level, total, total - 1
                ),
            )
            return None

        beam = filtered[index]
        print("[level={}] {}/{} | beam: {}".format(level, index + 1, total, beam.name))
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
    """Build a HOPSJob for *element* with explicit control over dispatch.

    Returns:
        job              — HOPSJob
        machining_report — list[str] comparing registered BTLx processings to
                           the machinings actually produced (one entry per
                           processing, plus a pass/fail summary line)
    """
    job = HOPSJob.from_element(element, scale_factor=scale_factor)
    rsi = job.ref_side_index
    opp_rsi = (rsi + 2) % 4

    pre_flip = []
    post_flip = []

    # Each entry: {"name": str, "rsi": int|str, "status": "ok"|"empty"|"skipped",
    #              "count": int, "placement": str, "ops": list[str]}
    conversion_log = []

    def _dispatch(name, rsi_proc, machinings, placement):
        ops = []
        for m in machinings:
            op = getattr(m, "OPERATION_TYPE", "?")
            tool = getattr(m, "tool", None)
            tool_name = type(tool).__name__ if tool is not None else "no-tool"
            ops.append("{} / {}".format(op, tool_name))
            m._processing_name = name  # tag for sorting
        status = "ok" if machinings else "empty"
        conversion_log.append(
            {
                "name": name,
                "rsi": rsi_proc,
                "status": status,
                "count": len(machinings),
                "placement": placement,
                "ops": ops,
            }
        )

    for processing in element.features:
        processing = processing.scaled(scale_factor)
        name = processing.PROCESSING_NAME
        rsi_proc = getattr(processing, "ref_side_index", "?")

        if name == "LongitudinalCut":
            machinings = LongitudinalCutStrategies.contouring(
                processing, machine_ref_side_index=rsi, tool=CastorD61()
            )
            post_flip.extend(machinings)
            _dispatch(name, rsi_proc, machinings, "post-flip")

        elif name == "Lap":
            machinings = LapStrategies.pocketing(
                processing, machine_ref_side_index=rsi, tool=CastorD61()
            )
            if (
                processing.ref_side_index == opp_rsi
            ):  # introduce a flip if the lap is on the opposite side
                pre_flip.extend(machinings)
                _dispatch(name, rsi_proc, machinings, "pre-flip")
            else:
                post_flip.extend(machinings)
                _dispatch(name, rsi_proc, machinings, "post-flip")

        elif name == "JackRafterCut":
            assert processing.ref_side_index in (rsi, opp_rsi), (
                f"Unexpected ref_side_index {processing.ref_side_index} for JackRafterCut"
            )
            if processing.ref_side_index == opp_rsi:
                machinings = JackRafterCutStrategies.sawing(
                    processing, machine_ref_side_index=opp_rsi, tool=SaegeD350()
                )
                pre_flip.extend(machinings)
                _dispatch(name, rsi_proc, machinings, "pre-flip")
            else:
                machinings = JackRafterCutStrategies.sawing(
                    processing, machine_ref_side_index=rsi, tool=SaegeD350()
                )
                post_flip.extend(machinings)
                _dispatch(name, rsi_proc, machinings, "post-flip")

        elif name == "Drilling":
            if processing.diameter == 4.0:
                if processing.ref_side_index == opp_rsi:
                    pre_flip.extend(
                        DrillingStrategies.pre_drilling(
                            processing,
                            machine_ref_side_index=opp_rsi,  # use opp_rsi to mill and add a 180 flip
                        )
                    )
                    _dispatch(name, rsi_proc, [], "pre-flip (pre-drill)")
                else:
                    machinings = DrillingStrategies.pre_drilling(
                        processing, machine_ref_side_index=rsi
                    )
                    # skip the drilling if it's too steep
                    if TOL.is_positive(abs(processing.inclination) - 20.0):
                        machinings.extend(
                            DrillingStrategies.drilling(
                                processing, machine_ref_side_index=rsi
                            )
                        )
                    post_flip.extend(machinings)
                    _dispatch(name, rsi_proc, machinings, "post-flip (pre-drill)")
            else:
                machinings = DrillingStrategies.pocketing(
                    processing, machine_ref_side_index=rsi, tool=SRSLD12()
                )
                if processing.ref_side_index == opp_rsi:
                    pre_flip.extend(machinings)
                    _dispatch(name, rsi_proc, machinings, "pre-flip")
                else:
                    post_flip.extend(machinings)
                    _dispatch(name, rsi_proc, machinings, "post-flip")

        elif name == "DoubleCut":
            strategy = processing.user_attributes.get("strategy", None)
            if strategy == "pocketing":
                if processing.ref_side_index == opp_rsi:
                    pre_flip.extend(
                        DoubleCutStrategies.pocketing(
                            processing,
                            machine_ref_side_index=opp_rsi,  # use opp_rsi to mill and add a 180 flip
                            first_cut=False,
                        )
                    )
                    _dispatch(
                        name,
                        rsi_proc,
                        [],
                        "pre-flip (strategy=pocketing, first_cut)",
                    )
                machinings = DoubleCutStrategies.pocketing(
                    processing,
                    machine_ref_side_index=rsi,
                    first_cut=False,
                )
            else:
                machinings = DoubleCutStrategies.milling(
                    processing,
                    machine_ref_side_index=rsi,
                    first_cut=False,
                    avoid_splintering=True,
                )
            post_flip.extend(machinings)
            _dispatch(
                name,
                rsi_proc,
                machinings,
                "post-flip (strategy={})".format(strategy or "milling"),
            )

        elif name == "BirdsMouth":
            machinings = BirdsMouthStrategies.milling(
                processing, machine_ref_side_index=rsi, avoid_splintering=True
            )
            post_flip.extend(machinings)
            _dispatch(name, rsi_proc, machinings, "post-flip")

        elif name == "StepJoint":
            machinings = StepJointStrategies.milling(
                processing, machine_ref_side_index=rsi
            )
            post_flip.extend(machinings)
            _dispatch(name, rsi_proc, machinings, "post-flip")

        else:
            conversion_log.append(
                {
                    "name": name,
                    "rsi": rsi_proc,
                    "status": "skipped",
                    "count": 0,
                    "placement": "-",
                    "ops": [],
                }
            )

    # Sort order: operation type → tool position → processing name
    # Lap is always last within each group.
    OP_ORDER = {"SAWING": 1, "DRILLING": 0, "MILLING": 2}
    PROCESSING_ORDER = {
        "Drilling": 0,
        "JackRafterCut": 1,
        "LongitudinalCut": 2,
        "DoubleCut": 2,
        "BirdsMouth": 3,
        "StepJoint": 8,  # always second last
        "Lap": 9,  # always last
    }

    def _sort_key(m):
        op_priority = OP_ORDER.get(getattr(m, "OPERATION_TYPE", ""), 3)
        tool = getattr(m, "tool", None)
        tool_position = getattr(tool, "position", 0) if tool is not None else 0
        processing_priority = PROCESSING_ORDER.get(
            getattr(m, "_processing_name", ""), 50
        )
        return (op_priority, tool_position, processing_priority)

    pre_flip.sort(key=_sort_key)
    post_flip.sort(key=_sort_key)

    if pre_flip:
        job.add(pre_flip)
        job.add(MachineStop(message="flip beam 180deg", park_mode=ParkMode.RIGHT_FRONT))
    job.add(post_flip)

    # --- Build comparison report ---
    n_total = len(conversion_log)
    n_ok = sum(1 for e in conversion_log if e["status"] == "ok")
    n_empty = sum(1 for e in conversion_log if e["status"] == "empty")
    n_skipped = sum(1 for e in conversion_log if e["status"] == "skipped")
    all_ok = n_ok == n_total

    STATUS_ICON = {"ok": "[OK]     ", "empty": "[EMPTY]  ", "skipped": "[SKIPPED]"}

    machining_report = [
        "=== Conversion report: {} (beam rsi={}) ===".format(element.name, rsi),
        "  Registered processings : {}".format(n_total),
        "  Converted (>0 machinings): {}".format(n_ok),
        "  Converted (0 machinings) : {}".format(n_empty),
        "  Skipped (no handler)    : {}".format(n_skipped),
        "  Result: {}".format(
            "ALL CONVERTED"
            if all_ok
            else "WARNING: {} processing(s) not fully converted".format(
                n_empty + n_skipped
            )
        ),
        "  " + "-" * 56,
    ]
    for i, entry in enumerate(conversion_log):
        icon = STATUS_ICON[entry["status"]]
        machining_report.append(
            "  {} #{:02d} {:22s} RSId:{} | {} | {} machining(s)".format(
                icon, i, entry["name"], entry["rsi"], entry["placement"], entry["count"]
            )
        )
        for op_str in entry["ops"]:
            machining_report.append("           -> {}".format(op_str))

    machining_report.append(
        "  pre-flip machinings: {}  |  post-flip machinings: {}".format(
            len(pre_flip), len(post_flip)
        )
    )

    return job, machining_report


def export_hop(beam, export_dir):
    """Write a .hop file for beam into export_dir/<beam.name>.hop.

    Returns:
        job              — HOPSJob
        machining_report — list[str] describing what each processing produced
    """
    os.makedirs(export_dir, exist_ok=True)
    hop_path = os.path.join(export_dir, beam.name + ".hop")
    job, machining_report = _element_to_job(beam)
    job.to_hop_file(hop_path)
    return machining_report


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------


def run(filepath, index, hierarchy, export, level=None, names=None):
    """Resolve one beam, visualise it, optionally export it, and return a processing report.

    Returns:
        geometry          — result of scene.draw()
        processing_report — list[str] of processing descriptions
    """
    # Load model
    model = load_model(filepath)
    export_dir = os.path.join(os.path.dirname(filepath), "hops")

    # Resolve beam
    beam = resolve_beam(model, index, hierarchy, level, names=names)
    if beam is None:
        return None, []

    # Override features
    override_features(beam)

    # Visualise geometry
    geometry = visualize_geometry(beam)

    # Export HOP
    machining_report = []
    if export:
        _, machining_report = export_hop(beam, export_dir)

    # Get processing report
    processing_report = get_processing_report(beam)

    return geometry, processing_report + ["-" * 40] + machining_report
