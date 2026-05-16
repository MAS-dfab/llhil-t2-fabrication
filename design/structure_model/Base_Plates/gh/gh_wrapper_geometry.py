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
- include_stiffeners
- enabled

GH Outputs:
- out
- payload
- members
- base_plates
- preview_breps
"""

import importlib.util
import os
import sys
import traceback

try:
    import Rhino  # type: ignore
except Exception:
    Rhino = None

ROOT = r"C:\Users\Juste\Documents\_GitHub\MAS-2526\10_llhil-t2-fabrication\design\structure_model\Base_Plates"
DEFAULT_PLATE_LENGTH = 0.8
DEFAULT_PLATE_WIDTH = 0.8
DEFAULT_PLATE_THICKNESS = 0.02
RUN_TAG = "BPG_WRAPPER_SYNC_2026_05_15_A"

# Diagnostic: print resolved path
print("[GH Wrapper] RUN_TAG:", RUN_TAG)
print("[GH Wrapper] ROOT:", ROOT)
HELPER_PATH = os.path.join(ROOT, "base_plate_geometry.py")
print("[GH Wrapper] HELPER_PATH:", HELPER_PATH)
print("[GH Wrapper] HELPER_EXISTS:", os.path.exists(HELPER_PATH))



def _load_helper():
    module_name = "base_plate_geometry"
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
    return module


def _mm_to_doc_scale():
    if Rhino is None:
        return 1.0
    try:
        doc = Rhino.RhinoDoc.ActiveDoc
        if doc is None:
            return 1.0
        return float(Rhino.RhinoMath.UnitScale(Rhino.UnitSystem.Millimeters, doc.ModelUnitSystem))
    except Exception:
        return 1.0


def _scale_breps_copy(breps, scale_factor):
    if Rhino is None:
        return [b for b in (breps or []) if b is not None]
    if abs(scale_factor - 1.0) <= 1e-12:
        return [b for b in (breps or []) if b is not None]

    scaled = []
    try:
        xform = Rhino.Geometry.Transform.Scale(Rhino.Geometry.Plane.WorldXY, scale_factor)
    except Exception:
        return [b for b in (breps or []) if b is not None]

    for brep in (breps or []):
        if brep is None:
            continue
        try:
            dup = brep.DuplicateBrep()
            if dup is None:
                continue
            dup.Transform(xform)
            scaled.append(dup)
        except Exception:
            scaled.append(brep)
    return scaled


try:
    helper = _load_helper()

    if "enabled" in globals() and enabled is False:
        print("[GH Wrapper] Branch: disabled")
        out = "Base plate geometry disabled."
        payload = {}
        members = []
        base_plates = []
        preview_breps = []
    else:
        print("[GH Wrapper] Branch: active")
        kwargs = {}
        if "payload_override" in globals() and isinstance(payload_override, dict) and payload_override:
            payload = payload_override
        else:
            kwargs = {
                "plate_length": DEFAULT_PLATE_LENGTH,
                "plate_width": DEFAULT_PLATE_WIDTH,
                "plate_thickness": DEFAULT_PLATE_THICKNESS,
                "bottom_face_mode": "Perpendicular_to_grain",
                "target_support_nodes_only": True,
                "deduplicate_support_nodes": True,
                "geometry_kind": "footing",
            }
            if "line_model_path" in globals() and line_model_path:
                from pathlib import Path
                kwargs["line_model_path"] = Path(str(line_model_path))
            else:
                # Fallback for GH context: use absolute path to repo's line_model data
                from pathlib import Path
                data_dir = Path(r"C:\Users\Juste\Documents\_GitHub\MAS-2526\10_llhil-t2-fabrication\design\line_model\data")
                # Prefer exports with known ground supports before meter-shifted variants.
                for candidate in ["0806_shifted_lines.json", "shifted_lines.json", "meters_shifted_lines.json"]:
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
            if "include_stiffeners" in globals() and include_stiffeners is not None:
                kwargs["include_stiffeners"] = bool(include_stiffeners)

            print("[GH Wrapper] kwargs:", kwargs)

            payload = helper.build_geometry_payload(**kwargs)
        members = payload.get("members", [])
        base_plates = payload.get("base_plates", [])
        footing_breps = payload.get("footing_breps", []) if isinstance(payload, dict) else []
        metadata = payload.get("metadata", {}) if isinstance(payload, dict) else {}
        print("[GH Wrapper] Extracted members: {0}".format(len(members)))
        print("[GH Wrapper] Extracted base_plates: {0}".format(len(base_plates)))
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
            mm_to_doc = _mm_to_doc_scale()
            preview_breps = _scale_breps_copy(footing_breps, mm_to_doc)
            if isinstance(payload, dict):
                payload["footing_breps"] = preview_breps
            print(
                "[GH Wrapper] Built {0} footing preview breps (mm->doc scale={1})".format(
                    len(preview_breps),
                    mm_to_doc,
                )
            )
        elif hasattr(helper, "BasePlateRecord") and base_plates:
            try:
                bp_records = [helper.BasePlateRecord(**bp) for bp in base_plates]
                preview_breps = helper.build_preview_breps(bp_records)
                preview_breps = [g for g in preview_breps if g is not None]
                print("[GH Wrapper] Built {0} preview breps".format(len(preview_breps)))
            except Exception as bp_err:
                print("[GH Wrapper] Error building preview breps: {0}".format(bp_err))
                import traceback
                print(traceback.format_exc())
        
        unit_hint = ""
        if members and base_plates:
            try:
                mw = float((members[0] or {}).get("width") or 0.0)
                pl = float((base_plates[0] or {}).get("length") or 0.0)
                if mw > 0.0 and pl / mw > 20.0:
                    unit_hint = " | WARNING: plate/member scale looks too large; check mm vs m"
            except Exception:
                pass
        out = "Loaded {0} members and built {1} base plates{2}".format(len(members), len(base_plates), unit_hint)
        print("[GH Wrapper] " + out)

except Exception:
    out = traceback.format_exc()
    payload = {}
    members = []
    base_plates = []
    preview_breps = []
    print("[GH Wrapper] Exception caught:")
    print(out)
