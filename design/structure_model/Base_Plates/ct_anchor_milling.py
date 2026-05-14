"""
CT anchor milling scaffold.

Consumes module (1) geometry payload and module (2) calculation payload,
extracts explicit milling geometry descriptors (holes + slot), and exports a
CT-ready JSON package.
"""

from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import uuid
from typing import Dict, List, Optional, Sequence, Tuple

try:
    import Rhino.Geometry as rg  # type: ignore
except Exception:
    rg = None


Point3 = Tuple[float, float, float]


def _to_point3(seq: Sequence[float]) -> Point3:
    return (float(seq[0]), float(seq[1]), float(seq[2]))


def _center_of_points(points: Sequence[Point3]) -> Point3:
    n = max(len(points), 1)
    sx = sum(p[0] for p in points)
    sy = sum(p[1] for p in points)
    sz = sum(p[2] for p in points)
    return (sx / n, sy / n, sz / n)


def _member_result_map(calc_payload: Dict[str, object]) -> Dict[str, Dict[str, object]]:
    out: Dict[str, Dict[str, object]] = {}
    for item in calc_payload.get("member_results", []):
        if isinstance(item, dict) and "member_id" in item:
            out[str(item["member_id"])] = item
    return out


def _member_map(geometry_payload: Dict[str, object]) -> Dict[str, Dict[str, object]]:
    out: Dict[str, Dict[str, object]] = {}
    for item in geometry_payload.get("members", []):
        if isinstance(item, dict) and "member_id" in item:
            out[str(item["member_id"])] = item
    return out


def _unit(v: Point3, fallback: Point3 = (1.0, 0.0, 0.0)) -> Point3:
    n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5
    if n <= 1e-9:
        return fallback
    return (v[0] / n, v[1] / n, v[2] / n)


def _cross(a: Point3, b: Point3) -> Point3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _add(a: Point3, b: Point3) -> Point3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(v: Point3, s: float) -> Point3:
    return (v[0] * s, v[1] * s, v[2] * s)


def _vector(a: Point3, b: Point3) -> Point3:
    return (b[0] - a[0], b[1] - a[1], b[2] - a[2])


def _box_brep(center: Point3, x_axis: Point3, y_axis: Point3, z_axis: Point3, lx: float, ly: float, lz: float):
    if rg is None:
        return None
    plane = rg.Plane(
        rg.Point3d(center[0], center[1], center[2]),
        rg.Vector3d(x_axis[0], x_axis[1], x_axis[2]),
        rg.Vector3d(y_axis[0], y_axis[1], y_axis[2]),
    )
    box = rg.Box(
        plane,
        rg.Interval(-0.5 * lx, 0.5 * lx),
        rg.Interval(-0.5 * ly, 0.5 * ly),
        rg.Interval(-0.5 * lz, 0.5 * lz),
    )
    return box.ToBrep()


def _cylinder_brep(start: Point3, end: Point3, radius: float):
    if rg is None:
        return None
    p0 = rg.Point3d(start[0], start[1], start[2])
    p1 = rg.Point3d(end[0], end[1], end[2])
    axis = p1 - p0
    length = axis.Length
    if length <= 1e-9 or radius <= 1e-9:
        return None
    mid = p0 + axis * 0.5
    plane = rg.Plane(mid, axis)
    cyl = rg.Cylinder(rg.Circle(plane, radius), length)
    return cyl.ToBrep(True, True)


def _hole_grid_centers(center: Point3, x_axis: Point3, y_axis: Point3, rows: int, holes_per_row: int, pitch: float, gage: float) -> List[Point3]:
    pts: List[Point3] = []
    hx = -0.5 * (max(holes_per_row, 1) - 1) * pitch
    hy = -0.5 * (max(rows, 1) - 1) * gage
    for i in range(max(holes_per_row, 1)):
        for j in range(max(rows, 1)):
            ox = hx + i * pitch
            oy = hy + j * gage
            pts.append(_add(center, _add(_scale(x_axis, ox), _scale(y_axis, oy))))
    return pts


def _member_frame(member: Dict[str, object], plate: Dict[str, object]) -> Tuple[Point3, Point3, Point3, Point3]:
    start_raw = member.get("start")
    end_raw = member.get("end")
    if isinstance(start_raw, Sequence) and len(start_raw) >= 3 and isinstance(end_raw, Sequence) and len(end_raw) >= 3:
        start = _to_point3(start_raw)
        end = _to_point3(end_raw)
        center = _center_of_points([start, end])
        x_axis = _unit(_vector(start, end), fallback=(1.0, 0.0, 0.0))
    else:
        center_raw = plate.get("center")
        center = _to_point3(center_raw) if isinstance(center_raw, Sequence) and len(center_raw) >= 3 else (0.0, 0.0, 0.0)
        x_axis = _unit(_to_point3(member.get("direction", (1.0, 0.0, 0.0))), fallback=(1.0, 0.0, 0.0))

    y_raw = plate.get("y_axis")
    if not (isinstance(y_raw, Sequence) and len(y_raw) >= 3):
        y_axis = _safe_y_axis(x_axis)
    else:
        y_axis = _unit(_to_point3(y_raw), fallback=_safe_y_axis(x_axis))

    z_axis = _cross(x_axis, y_axis)
    if abs(z_axis[0]) + abs(z_axis[1]) + abs(z_axis[2]) <= 1e-9:
        y_axis = _safe_y_axis(x_axis)
        z_axis = _cross(x_axis, y_axis)
    z_axis = _unit(z_axis, fallback=(0.0, 0.0, 1.0))
    y_axis = _unit(_cross(z_axis, x_axis), fallback=y_axis)
    return center, x_axis, y_axis, z_axis


def _safe_y_axis(x_axis: Point3) -> Point3:
    world_z = (0.0, 0.0, 1.0)
    y_axis = _cross(world_z, x_axis)
    if abs(y_axis[0]) + abs(y_axis[1]) + abs(y_axis[2]) <= 1e-9:
        y_axis = _cross((1.0, 0.0, 0.0), x_axis)
    return _unit(y_axis, fallback=(0.0, 1.0, 0.0))


def build_ct_records(
    geometry_payload: Dict[str, object],
    calc_payload: Dict[str, object],
    bottom_face_mode: str,
    process_only_passed: bool = False,
) -> List[Dict[str, object]]:
    member_results = _member_result_map(calc_payload)
    members = _member_map(geometry_payload)
    engineering = calc_payload.get("engineering") if isinstance(calc_payload.get("engineering"), dict) else {}
    sizing = engineering.get("sizing_recommendations") if isinstance(engineering, dict) and isinstance(engineering.get("sizing_recommendations"), dict) else {}
    fabrication = engineering.get("fabrication_parameters") if isinstance(engineering, dict) and isinstance(engineering.get("fabrication_parameters"), dict) else {}

    records: List[Dict[str, object]] = []
    for plate in calc_payload.get("adjusted_base_plates", []):
        if not isinstance(plate, dict):
            continue

        member_id = str(plate.get("member_id"))
        member = members.get(member_id, {})
        result = member_results.get(member_id, {})
        passed = bool(result.get("passed", False))

        if process_only_passed and not passed:
            continue

        corners = [_to_point3(c) for c in plate.get("corners_adjusted", plate.get("corners", []))]
        center = _center_of_points(corners) if corners else _to_point3(plate.get("center", (0.0, 0.0, 0.0)))
        x_axis = _unit(_to_point3(plate.get("x_axis", (1.0, 0.0, 0.0))), fallback=(1.0, 0.0, 0.0))
        y_axis = _unit(_to_point3(plate.get("y_axis", (0.0, 1.0, 0.0))), fallback=(0.0, 1.0, 0.0))
        normal = _unit(_to_point3(plate.get("normal", (0.0, 0.0, 1.0))), fallback=(0.0, 0.0, 1.0))

        rows = max(int(sizing.get("rows") or 2), 1)
        holes_per_row = max(int(sizing.get("holes_per_row") or 2), 1)
        pitch_parallel = float(sizing.get("pitch_parallel") or 100.0)
        gage_perp = float(sizing.get("gage_perp") or 60.0)
        hole_dia = float(sizing.get("bolt_hole_dia") or 0.0)
        slot_length = float(fabrication.get("slot_length") or sizing.get("plate_length") or 0.0)
        slot_width = float(fabrication.get("slot_width") or sizing.get("plate_thickness") or 0.0)
        slot_depth = float(fabrication.get("slot_depth") or sizing.get("plate_width") or 0.0)
        hole_depth = max(slot_depth, float(member.get("width") or 0.0), float(member.get("height") or 0.0), 60.0)

        hole_centers = _hole_grid_centers(
            center=center,
            x_axis=x_axis,
            y_axis=y_axis,
            rows=rows,
            holes_per_row=holes_per_row,
            pitch=pitch_parallel,
            gage=gage_perp,
        )

        holes: List[Dict[str, object]] = []
        for idx, hc in enumerate(hole_centers):
            hs = _add(hc, _scale(normal, -0.5 * hole_depth))
            he = _add(hc, _scale(normal, 0.5 * hole_depth))
            holes.append(
                {
                    "index": idx,
                    "type": "through_hole",
                    "center": hc,
                    "axis_start": hs,
                    "axis_end": he,
                    "diameter": hole_dia,
                    "depth": hole_depth,
                }
            )

        slot = {
            "type": "slot_cut",
            "center": center,
            "x_axis": x_axis,
            "y_axis": y_axis,
            "normal": normal,
            "length": slot_length,
            "width": slot_width,
            "depth": slot_depth,
        }

        _, timber_x, timber_y, timber_z = _member_frame(member, plate)
        start_raw = member.get("start")
        end_raw = member.get("end")
        start_pt = _to_point3(start_raw) if isinstance(start_raw, Sequence) and len(start_raw) >= 3 else center
        end_pt = _to_point3(end_raw) if isinstance(end_raw, Sequence) and len(end_raw) >= 3 else center
        timber_center = _center_of_points([start_pt, end_pt])
        timber_length = float(member.get("length") or 0.0)
        timber_width = float(member.get("width") or 0.0)
        timber_height = float(member.get("height") or 0.0)

        record = {
            "member_id": member_id,
            "member_index": int(plate.get("member_index") or 0),
            "group": plate.get("group"),
            "level": member.get("level"),
            "status": "PASS" if passed else "FAIL",
            "bottom_face_mode": bottom_face_mode,
            "adjustment_along_member": float(plate.get("adjustment_along_member") or 0.0),
            "timber": {
                "center": timber_center,
                "x_axis": timber_x,
                "y_axis": timber_y,
                "z_axis": timber_z,
                "start": start_pt,
                "end": end_pt,
                "length": timber_length,
                "width": timber_width,
                "height": timber_height,
            },
            "milling_geometry": {
                "holes": holes,
                "slot": slot,
            },
            "validation": result,
            "milling": {
                "anchor_pattern": "3x3_perimeter_optional_center_omit",
                "clearance_rule_mm": calc_payload.get("metadata", {}).get("min_allowable_clearance")
                if isinstance(calc_payload.get("metadata"), dict)
                else None,
                "bolt_hole_dia": sizing.get("bolt_hole_dia") if isinstance(sizing, dict) else None,
                "pitch_parallel": sizing.get("pitch_parallel") if isinstance(sizing, dict) else None,
                "gage_perp": sizing.get("gage_perp") if isinstance(sizing, dict) else None,
                "end_distance": sizing.get("end_distance") if isinstance(sizing, dict) else None,
                "edge_distance": sizing.get("edge_distance") if isinstance(sizing, dict) else None,
                "tolerances": {
                    "hole_clearance": fabrication.get("hole_clearance") if isinstance(fabrication, dict) else None,
                    "slot_clearance_each_side": fabrication.get("slot_clearance_each_side") if isinstance(fabrication, dict) else None,
                    "slot_extra_length": fabrication.get("slot_extra_length") if isinstance(fabrication, dict) else None,
                    "slot_extra_depth": fabrication.get("slot_extra_depth") if isinstance(fabrication, dict) else None,
                    "min_tool_clearance_preferred": fabrication.get("min_tool_clearance_preferred") if isinstance(fabrication, dict) else None,
                    "min_tool_clearance_absolute": fabrication.get("min_tool_clearance_absolute") if isinstance(fabrication, dict) else None,
                    "min_recess_edge_clear": fabrication.get("min_recess_edge_clear") if isinstance(fabrication, dict) else None,
                    "min_bolt_axis_to_obstruction": fabrication.get("min_bolt_axis_to_obstruction") if isinstance(fabrication, dict) else None,
                },
            },
        }
        records.append(record)

    records.sort(key=lambda x: int(x.get("member_index") or 0))
    return records


def build_timber_model_schema_block(ct_records: Sequence[Dict[str, object]]) -> Dict[str, object]:
    """
    Build a compas_timber-like model block aligned with timber_design JSON shape:
    {
      "data": {
        "elements": {
          "<guid>": {
             "data": {"edge": [...], "features": [], "frame": {...}, "group": ..., "height": ..., "length": ..., "level": ..., "width": ...},
             "dtype": "compas_timber.elements/Beam",
             "guid": "<guid>",
             "name": "base_plate_anchor"
          }
        }
      }
    }
    """
    elements: Dict[str, object] = {}

    for rec in ct_records:
        timber = rec.get("timber", {})
        milling_geometry = rec.get("milling_geometry", {})
        holes = milling_geometry.get("holes", []) if isinstance(milling_geometry, dict) else []
        slot = milling_geometry.get("slot") if isinstance(milling_geometry, dict) else None

        if not isinstance(timber, dict):
            continue

        member_id = str(rec.get("member_id"))
        member_index = int(rec.get("member_index") or 0)
        group = rec.get("group")
        level = rec.get("level")
        center = timber.get("center")
        x_axis = timber.get("x_axis")
        y_axis = timber.get("y_axis")

        if not (
            isinstance(center, Sequence)
            and len(center) >= 3
            and isinstance(x_axis, Sequence)
            and len(x_axis) >= 3
            and isinstance(y_axis, Sequence)
            and len(y_axis) >= 3
        ):
            continue

        c = (float(center[0]), float(center[1]), float(center[2]))
        x = _unit((float(x_axis[0]), float(x_axis[1]), float(x_axis[2])), fallback=(1.0, 0.0, 0.0))
        y = _unit((float(y_axis[0]), float(y_axis[1]), float(y_axis[2])), fallback=_safe_y_axis(x))

        width = float(timber.get("width") or 0.0) / 1000.0
        height = float(timber.get("height") or 0.0) / 1000.0
        length = float(timber.get("length") or 0.0) / 1000.0

        features: List[Dict[str, object]] = []
        for hole in holes if isinstance(holes, list) else []:
            if not isinstance(hole, dict):
                continue
            features.append(
                {
                    "dtype": "ct.milling/Hole",
                    "data": {
                        "center": hole.get("center"),
                        "axis_start": hole.get("axis_start"),
                        "axis_end": hole.get("axis_end"),
                        "diameter": hole.get("diameter"),
                        "depth": hole.get("depth"),
                    },
                }
            )
        if isinstance(slot, dict):
            features.append(
                {
                    "dtype": "ct.milling/Slot",
                    "data": {
                        "center": slot.get("center"),
                        "x_axis": slot.get("x_axis"),
                        "y_axis": slot.get("y_axis"),
                        "normal": slot.get("normal"),
                        "length": slot.get("length"),
                        "width": slot.get("width"),
                        "depth": slot.get("depth"),
                    },
                }
            )

        edge = [member_index * 2, member_index * 2 + 1]
        guid = str(uuid.uuid5(uuid.NAMESPACE_URL, "ct-anchor:" + member_id))
        elem = {
            "data": {
                "edge": edge,
                "features": features,
                "frame": {
                    "data": {
                        "point": [c[0], c[1], c[2]],
                        "xaxis": [x[0], x[1], x[2]],
                        "yaxis": [y[0], y[1], y[2]],
                    },
                    "dtype": "compas.geometry/Frame",
                    "guid": guid,
                },
                "group": int(group) if group is not None else None,
                "height": height,
                "length": length,
                "level": int(level) if level is not None else 0,
                "width": width,
            },
            "dtype": "compas_timber.elements/Beam",
            "guid": guid,
            "name": "column_with_milling",
        }
        elements[guid] = elem

    return {
        "data": {
            "elements": elements,
        }
    }


def build_inspection_breps(ct_records: Sequence[Dict[str, object]]) -> List[object]:
    if rg is None:
        return []

    breps: List[object] = []
    for rec in ct_records:
        timber = rec.get("timber", {})
        milling_geometry = rec.get("milling_geometry", {})
        if not isinstance(timber, dict) or not isinstance(milling_geometry, dict):
            continue

        center = timber.get("center")
        x_axis = timber.get("x_axis")
        y_axis = timber.get("y_axis")
        z_axis = timber.get("z_axis")
        length = float(timber.get("length") or 0.0)
        width = float(timber.get("width") or 0.0)
        height = float(timber.get("height") or 0.0)

        if not (
            isinstance(center, Sequence)
            and isinstance(x_axis, Sequence)
            and isinstance(y_axis, Sequence)
            and isinstance(z_axis, Sequence)
            and len(center) >= 3
            and len(x_axis) >= 3
            and len(y_axis) >= 3
            and len(z_axis) >= 3
        ):
            continue

        timber_brep = _box_brep(
            center=_to_point3(center),
            x_axis=_unit(_to_point3(x_axis), fallback=(1.0, 0.0, 0.0)),
            y_axis=_unit(_to_point3(y_axis), fallback=(0.0, 1.0, 0.0)),
            z_axis=_unit(_to_point3(z_axis), fallback=(0.0, 0.0, 1.0)),
            lx=length,
            ly=width,
            lz=height,
        )
        if timber_brep is None:
            continue

        cutters: List[object] = []
        holes = milling_geometry.get("holes", []) if isinstance(milling_geometry.get("holes"), list) else []
        for hole in holes:
            if not isinstance(hole, dict):
                continue
            hs = hole.get("axis_start")
            he = hole.get("axis_end")
            dia = float(hole.get("diameter") or 0.0)
            if isinstance(hs, Sequence) and isinstance(he, Sequence) and len(hs) >= 3 and len(he) >= 3 and dia > 0.0:
                hole_brep = _cylinder_brep(_to_point3(hs), _to_point3(he), 0.5 * dia)
                if hole_brep is not None:
                    cutters.append(hole_brep)

        slot = milling_geometry.get("slot")
        if isinstance(slot, dict):
            sc = slot.get("center")
            sx = slot.get("x_axis")
            sy = slot.get("y_axis")
            sn = slot.get("normal")
            sl = float(slot.get("length") or 0.0)
            sw = float(slot.get("width") or 0.0)
            sd = float(slot.get("depth") or 0.0)
            if (
                isinstance(sc, Sequence)
                and isinstance(sx, Sequence)
                and isinstance(sy, Sequence)
                and isinstance(sn, Sequence)
                and len(sc) >= 3
                and len(sx) >= 3
                and len(sy) >= 3
                and len(sn) >= 3
                and sl > 0.0
                and sw > 0.0
                and sd > 0.0
            ):
                slot_brep = _box_brep(
                    center=_to_point3(sc),
                    x_axis=_unit(_to_point3(sx), fallback=(1.0, 0.0, 0.0)),
                    y_axis=_unit(_to_point3(sy), fallback=(0.0, 1.0, 0.0)),
                    z_axis=_unit(_to_point3(sn), fallback=(0.0, 0.0, 1.0)),
                    lx=sl,
                    ly=sw,
                    lz=sd,
                )
                if slot_brep is not None:
                    cutters.append(slot_brep)

        current_parts = [timber_brep]
        for cutter in cutters:
            next_parts: List[object] = []
            for part in current_parts:
                diff = rg.Brep.CreateBooleanDifference(part, cutter, 0.01)
                if diff and len(diff) > 0:
                    next_parts.extend(diff)
                else:
                    next_parts.append(part)
            current_parts = next_parts

        breps.extend(current_parts)
    return breps


def export_ct_json(
    geometry_payload: Dict[str, object],
    calc_payload: Dict[str, object],
    out_json_path: Path,
    bottom_face_mode: Optional[str] = None,
    process_only_passed: bool = False,
) -> Dict[str, object]:
    effective_geometry_payload = geometry_payload
    maybe_synced = calc_payload.get("synced_geometry_payload")
    if isinstance(maybe_synced, dict) and maybe_synced:
        effective_geometry_payload = maybe_synced

    mode = bottom_face_mode or str(
        (calc_payload.get("metadata") or {}).get("bottom_face_mode")
        if isinstance(calc_payload.get("metadata"), dict)
        else "Perpendicular_to_grain"
    )

    records = build_ct_records(
        geometry_payload=effective_geometry_payload,
        calc_payload=calc_payload,
        bottom_face_mode=mode,
        process_only_passed=process_only_passed,
    )
    timber_model_schema = build_timber_model_schema_block(records)

    package = {
        "metadata": {
            "module": "ct_anchor_milling",
            "generated_utc": datetime.utcnow().isoformat() + "Z",
            "record_count": len(records),
            "bottom_face_mode": mode,
            "process_only_passed": process_only_passed,
            "geometry_source": (effective_geometry_payload.get("metadata") or {}).get("line_model_path")
            if isinstance(effective_geometry_payload.get("metadata"), dict)
            else None,
            "used_synced_geometry_payload": effective_geometry_payload is not geometry_payload,
            "ct_geometry_mode": "timber_columns_with_milling_cuts",
        },
        "records": records,
        "timber_model_schema": timber_model_schema,
    }

    out_json_path.parent.mkdir(parents=True, exist_ok=True)
    with out_json_path.open("w", encoding="utf-8") as fp:
        json.dump(package, fp, indent=2)

    return package


def _cli() -> None:
    import base_plate_geometry as geo
    import base_plate_calculations as calc

    g = geo.build_geometry_payload()
    c = calc.run_validation(g)
    out_path = Path(__file__).resolve().parent / "ct_anchor_milling_export.json"
    package = export_ct_json(g, c, out_path)
    print("Exported {0} records to {1}".format(len(package["records"]), out_path))


if __name__ == "__main__":
    _cli()
