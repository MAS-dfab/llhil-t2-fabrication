"""
Grasshopper Py3 wrapper: base_plate_geometry

GH Inputs expected:
- payload_override
- line_model_path
- plate_length
- plate_width
- plate_thickness
- bottom_face_mode
- include_hierarchies
- target_support_nodes_only
- support_z_tolerance
- deduplicate_support_nodes
- support_node_merge_tolerance
- target_member_index (deprecated alias for target_cluster_index)
- target_cluster_index
- geometry_kind
- baseplate_top_z
- base_plate_brep
- base_diameter
- min_hole_edge_spacing
- heel_fillet_radius
- timber_bottom_gap
- min_timber_gap
- bottom_end_distance_multiplier
- webplate_thickness
- bolt_dia
- hole_clearance
- total_bolt_count
- bolt_hole_dia
- webplate_hole_diameter
- webplate_hole_pitch
- webplate_hole_transverse_spacing
- webplate_hole_rows
- webplate_hole_pattern
- webplate_hole_stagger_offset
- stiffener_pair_axis_shift
- stiffener_pair_from_point
- stiffener_pair_to_point
- include_stiffeners
- sizing_recommendations
- calc_payload
- enabled

GH Outputs:
- out
- payload  (opaque payload token; keep downstream inputs on Item Access)
- members
- base_plates
- preview_breps
- footing_debug
"""

import importlib.util
import os
import sys
import traceback

try:
    import scriptcontext as sc  # type: ignore
except Exception:
    sc = None

try:
    import Rhino  # type: ignore
except Exception:
    Rhino = None

try:
    from Grasshopper.Kernel.Types import GH_ObjectWrapper  # type: ignore
except Exception:
    GH_ObjectWrapper = None

ROOT = r"C:\Users\Juste\Documents\_GitHub\MAS-2526\10_llhil-t2-fabrication\design\structure_model\Base_Plates"
DEFAULT_PLATE_LENGTH = 0.8
DEFAULT_PLATE_WIDTH = 0.8
DEFAULT_PLATE_THICKNESS = 0.02
DEFAULT_BASEPLATE_TOP_Z = 0.3658
RUN_TAG = "BPG_WRAPPER_SYNC_2026_05_19_DESIGNER_BOLT_INPUTS"

# Diagnostic: print resolved path
print("[GH Wrapper] RUN_TAG:", RUN_TAG)
print("[GH Wrapper] ROOT:", ROOT)
HELPER_PATH = os.path.join(ROOT, "py", "base_plate_geometry.py")
print("[GH Wrapper] HELPER_PATH:", HELPER_PATH)
print("[GH Wrapper] HELPER_EXISTS:", os.path.exists(HELPER_PATH))



def _load_helper():
    module_name = "base_plate_geometry"
    print("[GH Wrapper] HELPER_LOAD_START")
    sys.modules.pop(module_name, None)
    spec = importlib.util.spec_from_file_location(module_name, HELPER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError("Could not load {0}".format(HELPER_PATH))
    module = importlib.util.module_from_spec(spec)
    # Required for dataclasses/type resolution during module execution in GH Py3.
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception as e:
        print("[GH Wrapper] Error importing base_plate_geometry.py:", e)
        import traceback
        print(traceback.format_exc())
        raise
    print("[GH Wrapper] HELPER_LOADED: True")
    return module


def _unit_system_from_name(units_name):
    if Rhino is None:
        return None
    text = str(units_name or "").strip().lower()
    if text in ("m", "meter", "meters", "metre", "metres"):
        return Rhino.UnitSystem.Meters
    if text in ("mm", "millimeter", "millimeters", "millimetre", "millimetres"):
        return Rhino.UnitSystem.Millimeters
    return None


def _payload_to_doc_scale(payload_units):
    if Rhino is None:
        return 1.0
    try:
        doc = Rhino.RhinoDoc.ActiveDoc
        if doc is None:
            return 1.0
        source_units = _unit_system_from_name(payload_units)
        if source_units is None:
            return 1.0
        return float(Rhino.RhinoMath.UnitScale(source_units, doc.ModelUnitSystem))
    except Exception:
        return 1.0


def _scale_breps_copy(breps, scale_factor):
    if Rhino is None:
        return [b for b in (breps or []) if b is not None]
    if abs(scale_factor - 1.0) <= 1e-12:
        return [b for b in (breps or []) if b is not None]

    scaled = []
    transformed_count = 0
    passthrough_count = 0
    try:
        xform = Rhino.Geometry.Transform.Scale(Rhino.Geometry.Plane.WorldXY, scale_factor)
    except Exception:
        return [b for b in (breps or []) if b is not None]

    for geom in (breps or []):
        if geom is None:
            continue
        try:
            dup = None

            # Prefer geometry-preserving duplicate when available.
            duplicate = getattr(geom, "Duplicate", None)
            if callable(duplicate):
                dup = duplicate()

            # Fall back to Brep duplication path.
            if dup is None:
                duplicate_brep = getattr(geom, "DuplicateBrep", None)
                if callable(duplicate_brep):
                    dup = duplicate_brep()

            # Last resort: convert to Brep if possible.
            if dup is None:
                to_brep = getattr(geom, "ToBrep", None)
                if callable(to_brep):
                    try:
                        dup = to_brep(False)
                    except TypeError:
                        dup = to_brep()

            if dup is None:
                passthrough_count += 1
                scaled.append(geom)
                continue

            if not dup.Transform(xform):
                passthrough_count += 1
                scaled.append(geom)
                continue

            transformed_count += 1
            scaled.append(dup)
        except Exception:
            passthrough_count += 1
            scaled.append(geom)

    print(
        "[GH Wrapper] footing scale transform: transformed={0}, passthrough={1}".format(
            transformed_count,
            passthrough_count,
        )
    )
    return scaled


def _coerce_geometry_list(items):
    """Return only Rhino geometry objects, converting to Brep when possible."""
    coerced = []
    for item in (items or []):
        if item is None:
            continue
        geom = item
        if Rhino is not None:
            try:
                if isinstance(geom, Rhino.Geometry.GeometryBase):
                    coerced.append(geom)
                    continue
            except Exception:
                pass
        try:
            duplicate = getattr(item, "Duplicate", None)
            if callable(duplicate):
                geom = duplicate()
        except Exception:
            geom = item
        if Rhino is not None:
            try:
                if isinstance(geom, Rhino.Geometry.GeometryBase):
                    coerced.append(geom)
                    continue
            except Exception:
                pass
        try:
            to_brep = getattr(item, "ToBrep", None)
            if callable(to_brep):
                try:
                    brep = to_brep(False)
                except TypeError:
                    brep = to_brep()
                if brep is not None:
                    coerced.append(brep)
                    continue
        except Exception:
            pass
    return coerced


def _to_python_data(value):
    if isinstance(value, dict):
        return {key: _to_python_data(item) for key, item in value.items()}
    try:
        keys = list(value.Keys)
    except Exception:
        keys = None
    if keys is not None:
        return {
            _to_python_data(key): _to_python_data(value[key])
            for key in keys
        }
    if isinstance(value, (list, tuple)):
        return [_to_python_data(item) for item in value]
    return value


def _component_key():
    try:
        return str(ghenv.Component.InstanceGuid)  # type: ignore[name-defined]
    except Exception:
        return "standalone"


def _store_payload_reference(kind, value):
    if GH_ObjectWrapper is not None:
        print("[GH Wrapper] payload output mode=gh_object_wrapper kind={0}".format(kind))
        return GH_ObjectWrapper(value)
    if sc is None:
        return value
    token = "BPG_PAYLOAD::{0}::{1}".format(kind, _component_key())
    sc.sticky[token] = value
    print("[GH Wrapper] payload output mode=reference_token kind={0}".format(kind))
    return token


def _unwrap_payload(value):
    normalized = value
    while True:
        wrapped_value = getattr(normalized, "Value", None)
        if wrapped_value is not None and wrapped_value is not normalized:
            normalized = wrapped_value
            continue
        branches = getattr(normalized, "Branches", None)
        if branches is not None:
            normalized = _to_python_data(branches)
            continue
        normalized = _to_python_data(normalized)
        if isinstance(normalized, str) and sc is not None and normalized in sc.sticky:
            normalized = sc.sticky[normalized]
            continue
        if isinstance(normalized, (list, tuple)) and len(normalized) == 1:
            normalized = normalized[0]
            continue
        return normalized


def _coerce_webplate_hole_rows(value):
    """Accept both the intended 1/2 integer control and a GH bool toggle."""
    if isinstance(value, bool):
        return 2 if value else 1
    text = str(value).strip().lower()
    if text == "true":
        return 2
    if text == "false":
        return 1
    try:
        numeric = int(value)
    except Exception:
        return None
    if numeric in (1, 2):
        return numeric
    if numeric == 0:
        return 1
    return 2 if numeric > 1 else 1


def _coerce_webplate_hole_pattern(value):
    try:
        numeric = float(value)
        text = str(int(numeric)) if numeric.is_integer() else str(value).strip().lower()
    except Exception:
        text = str(value).strip().lower()
    text = text.replace(" ", "_")
    aliases = {
        "1": "single_row_centerline",
        "single": "single_row_centerline",
        "single_row": "single_row_centerline",
        "single_row_centerline": "single_row_centerline",
        "2": "double_row",
        "double": "double_row",
        "double_row": "double_row",
        "rectangular": "double_row",
        "3": "staggered_double_row",
        "stagger": "staggered_double_row",
        "staggered": "staggered_double_row",
        "staggered_double_row": "staggered_double_row",
    }
    return aliases.get(text)


print("[GH Wrapper] BODY_READY")


try:
    helper = _load_helper()
    try:
        print(
            "[GH Wrapper] helper capabilities: rg_available={0}, base_footing_run_callable={1}".format(
                bool(getattr(helper, "rg", None) is not None),
                callable(getattr(helper, "base_footing_run", None)),
            )
        )
    except Exception:
        pass

    if "enabled" in globals() and enabled is False:
        print("[GH Wrapper] Branch: disabled")
        out = "Base plate geometry disabled."
        payload = {}
        members = []
        base_plates = []
        preview_breps = []
        footing_debug = []
    else:
        print("[GH Wrapper] Branch: active")
        kwargs = {}
        if "payload_override" in globals() and payload_override:
            payload_candidate = _unwrap_payload(payload_override)
            payload = payload_candidate if isinstance(payload_candidate, dict) else payload_override
        else:
            kwargs = {
                "plate_length": DEFAULT_PLATE_LENGTH,
                "plate_width": DEFAULT_PLATE_WIDTH,
                "plate_thickness": DEFAULT_PLATE_THICKNESS,
                "bottom_face_mode": "Perpendicular_to_grain",
                "target_support_nodes_only": True,
                "deduplicate_support_nodes": True,
                "geometry_kind": "footing",
                "baseplate_top_z": DEFAULT_BASEPLATE_TOP_Z,
            }
            if "line_model_path" in globals() and line_model_path:
                from pathlib import Path
                kwargs["line_model_path"] = Path(str(line_model_path))
            else:
                # Fallback for GH context: use absolute path to repo's line_model data
                from pathlib import Path
                data_dir = Path(r"C:\Users\Juste\Documents\_GitHub\MAS-2526\10_llhil-t2-fabrication\design\line_model\data")
                # Prefer the latest explicit line-model export before legacy variants.
                for candidate in ["260516_v1_line_model.json", "0806_shifted_lines.json", "shifted_lines.json", "meters_shifted_lines.json"]:
                    candidate_path = data_dir / candidate
                    if candidate_path.exists():
                        kwargs["line_model_path"] = candidate_path
                        break
            if "plate_length" in globals() and plate_length is not None:
                kwargs["plate_length"] = float(plate_length)
            if "plate_width" in globals() and plate_width is not None:
                kwargs["plate_width"] = float(plate_width)
            if "plate_thickness" in globals() and plate_thickness is not None:
                kwargs["plate_thickness"] = float(plate_thickness)
            if "bottom_face_mode" in globals() and bottom_face_mode:
                kwargs["bottom_face_mode"] = str(bottom_face_mode)
            if "include_hierarchies" in globals() and include_hierarchies:
                kwargs["include_hierarchies"] = include_hierarchies
            if "target_support_nodes_only" in globals() and target_support_nodes_only is not None:
                kwargs["target_support_nodes_only"] = bool(target_support_nodes_only)
            if "support_z_tolerance" in globals() and support_z_tolerance is not None:
                kwargs["support_z_tolerance"] = float(support_z_tolerance)
            if "deduplicate_support_nodes" in globals() and deduplicate_support_nodes is not None:
                kwargs["deduplicate_support_nodes"] = bool(deduplicate_support_nodes)
            if "support_node_merge_tolerance" in globals() and support_node_merge_tolerance is not None:
                kwargs["support_node_merge_tolerance"] = float(support_node_merge_tolerance)
            if "target_cluster_index" in globals() and target_cluster_index is not None:
                kwargs["target_cluster_index"] = int(target_cluster_index)
            elif "target_member_index" in globals() and target_member_index is not None:
                # Backward compatibility only: use target_cluster_index (1..4) going forward.
                kwargs["target_cluster_index"] = int(target_member_index)
                print("[GH Wrapper] target_member_index is deprecated; use target_cluster_index (1..4).")
            if "geometry_kind" in globals() and geometry_kind:
                kwargs["geometry_kind"] = str(geometry_kind)
            if "baseplate_top_z" in globals() and baseplate_top_z is not None:
                kwargs["baseplate_top_z"] = float(baseplate_top_z)
            if "base_plate_brep" in globals() and base_plate_brep is not None:
                kwargs["base_plate_brep"] = base_plate_brep
            if "base_diameter" in globals() and base_diameter is not None:
                kwargs["base_diameter"] = float(base_diameter)
            if "min_hole_edge_spacing" in globals() and min_hole_edge_spacing is not None:
                kwargs["min_hole_edge_spacing"] = float(min_hole_edge_spacing)
            if "heel_fillet_radius" in globals() and heel_fillet_radius is not None:
                kwargs["heel_fillet_radius"] = float(heel_fillet_radius)
            if "timber_bottom_gap" in globals() and timber_bottom_gap is not None:
                kwargs["timber_bottom_gap"] = float(timber_bottom_gap)
            if "min_timber_gap" in globals() and min_timber_gap is not None:
                kwargs["min_timber_gap"] = float(min_timber_gap)
            if "bottom_end_distance_multiplier" in globals() and bottom_end_distance_multiplier is not None:
                kwargs["bottom_end_distance_multiplier"] = float(bottom_end_distance_multiplier)
            if "webplate_thickness" in globals() and webplate_thickness is not None:
                kwargs["webplate_thickness"] = float(webplate_thickness)
            if "bolt_dia" in globals() and bolt_dia is not None:
                kwargs["bolt_dia"] = float(bolt_dia)
                print("[GH Wrapper] designer bolt_dia applied:", kwargs["bolt_dia"])
            if "hole_clearance" in globals() and hole_clearance is not None:
                kwargs["hole_clearance"] = float(hole_clearance)
                print("[GH Wrapper] designer hole_clearance applied:", kwargs["hole_clearance"])
            total_bolt_count_input_name = None
            total_bolt_count_input_value = None
            for candidate_name in (
                "total_bolt_count",
                "webplate_bolt_count",
                "bolt_count",
                "n_bolts",
            ):
                if candidate_name in globals() and globals().get(candidate_name) is not None:
                    total_bolt_count_input_name = candidate_name
                    total_bolt_count_input_value = globals().get(candidate_name)
                    break
            if total_bolt_count_input_name is not None:
                kwargs["total_bolt_count"] = int(float(total_bolt_count_input_value))
                print(
                    "[GH Wrapper] designer bolt count {0}={1!r}".format(
                        total_bolt_count_input_name,
                        total_bolt_count_input_value,
                    )
                )
            if "bolt_hole_dia" in globals() and bolt_hole_dia is not None:
                kwargs["webplate_hole_diameter"] = float(bolt_hole_dia)
                print("[GH Wrapper] bolt_hole_dia override applied:", kwargs["webplate_hole_diameter"])
            elif "webplate_hole_diameter" in globals() and webplate_hole_diameter is not None:
                kwargs["webplate_hole_diameter"] = float(webplate_hole_diameter)
                print("[GH Wrapper] webplate_hole_diameter alias applied:", kwargs["webplate_hole_diameter"])
            if "webplate_hole_pitch" in globals() and webplate_hole_pitch is not None:
                kwargs["webplate_hole_pitch"] = float(webplate_hole_pitch)
            if "webplate_hole_transverse_spacing" in globals() and webplate_hole_transverse_spacing is not None:
                kwargs["webplate_hole_transverse_spacing"] = float(webplate_hole_transverse_spacing)
            row_input_name = None
            row_input_value = None
            if "webplate_hole_rows" in globals() and webplate_hole_rows is not None:
                row_input_name = "webplate_hole_rows"
                row_input_value = webplate_hole_rows
            elif "plate_hole_rows" in globals() and plate_hole_rows is not None:
                row_input_name = "plate_hole_rows"
                row_input_value = plate_hole_rows
            if row_input_name is not None:
                resolved_rows = _coerce_webplate_hole_rows(row_input_value)
                if resolved_rows is not None:
                    kwargs["webplate_hole_rows"] = resolved_rows
                    print(
                        "[GH Wrapper] hole row input {0}={1!r} -> rows={2}".format(
                            row_input_name,
                            row_input_value,
                            resolved_rows,
                        )
                    )
                    if row_input_name == "plate_hole_rows":
                        print("[GH Wrapper] plate_hole_rows is a supported alias; prefer webplate_hole_rows.")
            pattern_input_name = None
            pattern_input_value = None
            if "webplate_hole_pattern" in globals() and webplate_hole_pattern is not None:
                pattern_input_name = "webplate_hole_pattern"
                pattern_input_value = webplate_hole_pattern
            elif "plate_hole_pattern" in globals() and plate_hole_pattern is not None:
                pattern_input_name = "plate_hole_pattern"
                pattern_input_value = plate_hole_pattern
            elif (
                row_input_name is not None
                and _coerce_webplate_hole_pattern(row_input_value) == "staggered_double_row"
            ):
                pattern_input_name = row_input_name
                pattern_input_value = row_input_value
            if pattern_input_name is not None:
                resolved_pattern = _coerce_webplate_hole_pattern(pattern_input_value)
                if resolved_pattern is not None:
                    kwargs["webplate_hole_pattern"] = resolved_pattern
                    if resolved_pattern == "staggered_double_row":
                        kwargs["webplate_hole_rows"] = 2
                    print(
                        "[GH Wrapper] hole pattern input {0}={1!r} -> pattern={2}".format(
                            pattern_input_name,
                            pattern_input_value,
                            resolved_pattern,
                        )
                    )
            if "webplate_hole_stagger_offset" in globals() and webplate_hole_stagger_offset is not None:
                kwargs["webplate_hole_stagger_offset"] = float(webplate_hole_stagger_offset)

            pair_axis_shift_input_name = None
            pair_axis_shift_input_value = None
            for candidate_name in (
                "stiffener_pair_axis_shift",
                "stiffener_pair_shift",
                "stiffener_pair_translate",
            ):
                if candidate_name in globals() and globals().get(candidate_name) is not None:
                    pair_axis_shift_input_name = candidate_name
                    pair_axis_shift_input_value = globals().get(candidate_name)
                    break
            if pair_axis_shift_input_name is not None:
                kwargs["stiffener_pair_axis_shift"] = float(pair_axis_shift_input_value)
                print(
                    "[GH Wrapper] stiffener pair axis shift {0}={1!r}".format(
                        pair_axis_shift_input_name,
                        pair_axis_shift_input_value,
                    )
                )

            pair_from_input_name = None
            pair_from_input_value = None
            for candidate_name in (
                "stiffener_pair_from_point",
                "stiffener_point_1",
                "stiffener_point1",
            ):
                if candidate_name in globals() and globals().get(candidate_name) is not None:
                    pair_from_input_name = candidate_name
                    pair_from_input_value = globals().get(candidate_name)
                    break
            if pair_from_input_name is not None:
                kwargs["stiffener_pair_from_point"] = pair_from_input_value
                print("[GH Wrapper] stiffener pair from point input: {0}".format(pair_from_input_name))

            pair_to_input_name = None
            pair_to_input_value = None
            for candidate_name in (
                "stiffener_pair_to_point",
                "stiffener_point_2",
                "stiffener_point2",
            ):
                if candidate_name in globals() and globals().get(candidate_name) is not None:
                    pair_to_input_name = candidate_name
                    pair_to_input_value = globals().get(candidate_name)
                    break
            if pair_to_input_name is not None:
                kwargs["stiffener_pair_to_point"] = pair_to_input_value
                print("[GH Wrapper] stiffener pair to point input: {0}".format(pair_to_input_name))

            if "include_stiffeners" in globals() and include_stiffeners is not None:
                kwargs["include_stiffeners"] = bool(include_stiffeners)
            if "sizing_recommendations" in globals() and sizing_recommendations:
                kwargs["sizing_recommendations"] = _unwrap_payload(sizing_recommendations)
            elif "calc_payload" in globals() and calc_payload:
                kwargs["sizing_recommendations"] = _unwrap_payload(calc_payload)

            print("[GH Wrapper] kwargs:", kwargs)

            payload = helper.build_geometry_payload(**kwargs)
        members = payload.get("members", [])
        base_plate_records = payload.get("base_plates", [])
        base_plates = []
        footing_breps = payload.get("footing_breps", []) if isinstance(payload, dict) else []
        metadata = payload.get("metadata", {}) if isinstance(payload, dict) else {}
        handoff = payload.get("handoff", {}) if isinstance(payload, dict) else {}
        footing_debug_records = metadata.get("footing_debug", []) if isinstance(metadata, dict) else []
        footing_debug = [repr(item) for item in footing_debug_records]
        metadata_member_count = metadata.get("member_count") if isinstance(metadata, dict) else None
        metadata_geometry_kind = metadata.get("geometry_kind") if isinstance(metadata, dict) else None
        metadata_footing_handoff_count = metadata.get("footing_handoff_count") if isinstance(metadata, dict) else None
        print("[GH Wrapper] Extracted members: {0}".format(len(members)))
        print("[GH Wrapper] Extracted base_plates: {0}".format(len(base_plate_records)))
        if metadata_member_count is not None:
            print(
                "[GH Wrapper] metadata summary: member_count={0}, geometry_kind={1}, footing_handoff_count={2}".format(
                    metadata_member_count,
                    metadata_geometry_kind,
                    metadata_footing_handoff_count,
                )
            )
        if metadata.get("support_cluster_member_indices"):
            print(
                "[GH Wrapper] support cluster member indices: {0}".format(
                    metadata.get("support_cluster_member_indices")
                )
            )
        if footing_debug_records:
            first_debug = footing_debug_records[0] if isinstance(footing_debug_records[0], dict) else {}
            stiffener_debug = first_debug.get("stiffener_specs") if isinstance(first_debug, dict) else []
            if stiffener_debug:
                print(
                    "[GH Wrapper] stiffeners: count={0}, kinds={1}".format(
                        len(stiffener_debug),
                        [item.get("stiffener_kind") for item in stiffener_debug],
                    )
                )
                face_stiffener_shifts = [
                    {
                        "member": item.get("target_anchor_member_id"),
                        "face_depth": item.get("webplate_depth"),
                        "target_source": item.get("webplate_intersection_source"),
                        "collision_point": item.get("collision_point_input"),
                        "plane_projection": item.get("projected_collision_point"),
                        "edge_projection": item.get("edge_projected_collision_point"),
                        "edge_name": item.get("edge_projected_collision_name"),
                        "heel_face_start": item.get("heel_face_midline_start"),
                        "heel_face_end": item.get("heel_face_midline_end"),
                        "heel_face_target": item.get("heel_face_midpoint_at_timber_face"),
                        "heel_face_t": item.get("heel_face_intersection_param"),
                        "target": item.get("webplate_intersection_target"),
                        "from_midpoint": item.get("unsnapped_pair_shared_edge_midpoint"),
                        "pair_snap": item.get("pair_snap_vector"),
                        "side_span": item.get("side_mount_span"),
                        "side_offset": item.get("side_mount_center_offset"),
                        "heel_y": item.get("heel_side_alignment_y"),
                        "toe_y": item.get("toe_side_alignment_y"),
                        "center_shift": item.get("bottom_face_center_shift"),
                        "pair_axis_shift": item.get("pair_axis_shift"),
                        "pair_axis_shift_source": item.get("pair_axis_shift_source"),
                        "pair_axis_shift_vector": item.get("pair_axis_shift_vector"),
                    }
                    for item in stiffener_debug
                    if item.get("stiffener_kind") == "timber_bottom_face"
                ]
                if face_stiffener_shifts:
                    print(
                        "[GH Wrapper] timber-face stiffener edge alignment: {0}".format(
                            face_stiffener_shifts
                        )
                    )
        milling_metadata = handoff.get("milling", {}) if isinstance(handoff, dict) else {}
        if isinstance(milling_metadata, dict) and milling_metadata.get("bolt_hole_diameter") is not None:
            print(
                "[GH Wrapper] web plate holes: rows={0}, per_row={1}, total={2}, mode={3}, dia_mm={4}, gage_mm={5}, pitch_mm={6}, source={7}".format(
                    milling_metadata.get("plate_hole_rows"),
                    milling_metadata.get("plate_holes_per_row"),
                    milling_metadata.get("plate_total_hole_count"),
                    milling_metadata.get("plate_hole_row_mode"),
                    None if milling_metadata.get("bolt_hole_diameter") is None else 1000.0 * float(milling_metadata.get("bolt_hole_diameter")),
                    None if milling_metadata.get("gage_perp") is None else 1000.0 * float(milling_metadata.get("gage_perp")),
                    None if milling_metadata.get("pitch_parallel") is None else 1000.0 * float(milling_metadata.get("pitch_parallel")),
                    milling_metadata.get("bolt_hole_dimension_source"),
                )
            )
            if milling_metadata.get("plate_hole_center_counts"):
                print(
                    "[GH Wrapper] web plate hole center counts per plate: {0}".format(
                        milling_metadata.get("plate_hole_center_counts")
                    )
                )
            if milling_metadata.get("plate_hole_pattern_diagnostics"):
                print(
                    "[GH Wrapper] web plate hole pattern diagnostics: {0}".format(
                        [
                            {
                                "plate_index": item.get("plate_index"),
                                "expected": item.get("expected_count"),
                                "generated": item.get("generated_count"),
                                "inside": item.get("inside_plate_count"),
                                "status": item.get("status"),
                            }
                            for item in milling_metadata.get("plate_hole_pattern_diagnostics")
                        ]
                    )
                )
        geometry_metadata = handoff.get("geometry", {}) if isinstance(handoff, dict) else {}
        if isinstance(geometry_metadata, dict) and geometry_metadata.get("stiffener_specs"):
            print(
                "[GH Wrapper] stiffener targets: {0}".format(
                    [
                        {
                            "source": item.get("target_source"),
                            "member_id": item.get("target_anchor_member_id"),
                            "member_index": item.get("target_anchor_member_index"),
                            "azimuth_deg": item.get("azimuth_deg"),
                        }
                        for item in geometry_metadata.get("stiffener_specs")
                    ]
                )
            )
        if footing_debug_records:
            print("[GH Wrapper] footing debug: {0}".format(footing_debug_records))
        if "target_cluster_index" in kwargs:
            print(
                "[GH Wrapper] target_cluster_index={0}, mode={1}".format(
                    kwargs.get("target_cluster_index"),
                    metadata.get("target_member_index_mode"),
                )
            )
            try:
                available_indices = [int((m or {}).get("index")) for m in members if isinstance(m, dict) and (m or {}).get("index") is not None]
                print("[GH Wrapper] available member indices: {0}".format(available_indices))
            except Exception:
                pass
        
        preview_breps = []
        if footing_breps:
            footing_units = metadata.get("footing_output_units") or metadata.get("geometry_units") or "millimeters"
            payload_to_doc = _payload_to_doc_scale(footing_units)
            _scaled_breps = _scale_breps_copy(footing_breps, payload_to_doc)
            preview_breps = _coerce_geometry_list(_scaled_breps)
            if isinstance(payload, dict):
                payload["preview"] = {
                    "breps_doc_units": preview_breps,
                    "source_units": footing_units,
                    "document_scale_factor": payload_to_doc,
                }
            print(
                "[GH Wrapper] Built {0} footing preview breps ({1}->doc scale={2})".format(
                    len(preview_breps),
                    footing_units,
                    payload_to_doc,
                )
            )
            if len(preview_breps) != len(_scaled_breps):
                print("[GH Wrapper] Dropped {0} non-geometry footing items during coercion".format(
                    len(_scaled_breps) - len(preview_breps)
                ))
        elif metadata_geometry_kind == "footing":
            print(
                "[GH Wrapper] No footing_breps in payload. diagnostics: helper_rg_available={0}, helper_base_footing_run_callable={1}, member_count={2}, footing_handoff_count={3}".format(
                    bool(getattr(helper, "rg", None) is not None),
                    callable(getattr(helper, "base_footing_run", None)),
                    len(members),
                    metadata_footing_handoff_count,
                )
            )
        elif hasattr(helper, "BasePlateRecord") and base_plate_records:
            try:
                bp_records = [helper.BasePlateRecord(**bp) for bp in base_plate_records]
                preview_breps = _coerce_geometry_list(helper.build_preview_breps(bp_records))
                print("[GH Wrapper] Built {0} preview breps".format(len(preview_breps)))
            except Exception as bp_err:
                print("[GH Wrapper] Error building preview breps: {0}".format(bp_err))
                import traceback
                print(traceback.format_exc())

        # GH output "base_plates" is commonly wired to geometry params; emit breps
        # directly to avoid Goo->Geometry conversion failures on dict records.
        if preview_breps:
            base_plates = _coerce_geometry_list(preview_breps)
            if isinstance(payload, dict):
                payload["base_plate_records"] = base_plate_records
            print("[GH Wrapper] base_plates output mapped to preview breps: {0}".format(len(base_plates)))
        else:
            base_plates = []
        
        unit_hint = ""
        if members and base_plate_records:
            try:
                mw = float((members[0] or {}).get("width") or 0.0)
                pl = float((base_plate_records[0] or {}).get("length") or 0.0)
                if mw > 0.0 and pl / mw > 20.0:
                    unit_hint = " | WARNING: plate/member scale looks too large; check mm vs m"
            except Exception:
                pass
        out = "Loaded {0} members and built {1} base plate records; output {2} preview breps{3}".format(
            len(members),
            len(base_plate_records),
            len(preview_breps),
            unit_hint,
        )
        payload = _store_payload_reference("geometry", payload)
        print("[GH Wrapper] " + out)

except Exception:
    out = traceback.format_exc()
    payload = {}
    members = []
    base_plates = []
    preview_breps = []
    footing_debug = []
    print("[GH Wrapper] Exception caught:")
    print(out)
