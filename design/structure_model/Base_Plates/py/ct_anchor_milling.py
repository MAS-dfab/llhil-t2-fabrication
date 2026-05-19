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
SUPPORTED_GEOMETRY_UNITS = {"meters", "millimeters"}


def _canonical_units(value: object, default: str = "millimeters") -> str:
    if value is None:
        return default
    text = str(value).strip().lower()
    aliases = {
        "m": "meters",
        "meter": "meters",
        "meters": "meters",
        "metre": "meters",
        "metres": "meters",
        "mm": "millimeters",
        "millimeter": "millimeters",
        "millimeters": "millimeters",
        "millimetre": "millimeters",
        "millimetres": "millimeters",
    }
    return aliases.get(text, default)


def _unit_scale_factor(from_units: str, to_units: str) -> float:
    source = _canonical_units(from_units, from_units)
    target = _canonical_units(to_units, to_units)
    if source == target:
        return 1.0
    if source == "meters" and target == "millimeters":
        return 1000.0
    if source == "millimeters" and target == "meters":
        return 0.001
    return 1.0


def _median(values: Sequence[float], default: float = 0.0) -> float:
    items = sorted(float(value) for value in values)
    if not items:
        return default
    middle = len(items) // 2
    if len(items) % 2:
        return items[middle]
    return 0.5 * (items[middle - 1] + items[middle])


def _geometry_payload_units(payload: Dict[str, object]) -> str:
    metadata = payload.get("metadata")
    if isinstance(metadata, dict):
        explicit = _canonical_units(metadata.get("geometry_units"), "")
        if explicit in SUPPORTED_GEOMETRY_UNITS:
            return explicit

    widths: List[float] = []
    for member in payload.get("members", []):
        if isinstance(member, dict) and member.get("width") is not None:
            widths.append(abs(float(member["width"])))
    median_width = _median(widths, 0.0)
    return "meters" if 0.0 < median_width < 10.0 else "millimeters"


def _scale_point_units(point: Point3, from_units: str, to_units: str) -> Point3:
    factor = _unit_scale_factor(from_units, to_units)
    return (point[0] * factor, point[1] * factor, point[2] * factor)


def _scale_optional_point(value: object, from_units: str, to_units: str) -> object:
    point = _point3_or_none(value)
    if point is not None:
        return _scale_point_units(point, from_units, to_units)
    return value


def _scale_optional_dimension(value: object, from_units: str, to_units: str) -> object:
    if value is None:
        return value
    return float(value) * _unit_scale_factor(from_units, to_units)


def _boolean_tolerance(units: str) -> float:
    """Return the CT boolean tolerance as 0.01 mm in the active units."""
    return 0.01 * _unit_scale_factor("millimeters", units)


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


def _member_index(member: Dict[str, object]) -> Optional[int]:
    raw = member.get("member_index")
    if raw is None:
        raw = member.get("index")
    try:
        return int(raw) if raw is not None else None
    except Exception:
        return None


def _footing_record_map(geometry_payload: Dict[str, object]) -> Dict[str, Dict[str, object]]:
    out: Dict[str, Dict[str, object]] = {}
    for item in geometry_payload.get("footings", []):
        if isinstance(item, dict) and item.get("member_id") is not None:
            out[str(item["member_id"])] = item
    return out


def _list_item(value: object, index: int) -> object:
    if isinstance(value, list) and 0 <= index < len(value):
        return value[index]
    if isinstance(value, tuple) and 0 <= index < len(value):
        return value[index]
    return None


def _item_for_member_index(
    value: object,
    member_index: Optional[int],
    fallback_index: int,
) -> object:
    items = value if isinstance(value, (list, tuple)) else []
    if member_index is not None:
        for item in items:
            if isinstance(item, dict):
                raw = item.get("member_index")
                try:
                    if raw is not None and int(raw) == member_index:
                        return item
                except Exception:
                    pass
    return _list_item(items, fallback_index)


def _point3_or_none(value: object) -> Optional[Point3]:
    if isinstance(value, dict):
        for keys in (("x", "y", "z"), ("X", "Y", "Z")):
            if all(k in value for k in keys):
                try:
                    return (float(value[keys[0]]), float(value[keys[1]]), float(value[keys[2]]))
                except Exception:
                    return None
    for keys in (("X", "Y", "Z"), ("x", "y", "z")):
        if all(hasattr(value, k) for k in keys):
            try:
                return (
                    float(getattr(value, keys[0])),
                    float(getattr(value, keys[1])),
                    float(getattr(value, keys[2])),
                )
            except Exception:
                return None
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)) and len(value) >= 3:
        return _to_point3(value)
    return None


def _frame_from_ref(
    frame_ref: object,
    fallback_origin: Point3,
    fallback_x: Point3,
    fallback_y: Point3,
    fallback_normal: Point3,
) -> Tuple[Point3, Point3, Point3, Point3]:
    ref = frame_ref if isinstance(frame_ref, dict) else {}
    origin = _point3_or_none(ref.get("origin")) or fallback_origin
    x_axis = _unit(_point3_or_none(ref.get("x_axis")) or fallback_x, fallback=fallback_x)
    y_axis = _unit(_point3_or_none(ref.get("y_axis")) or fallback_y, fallback=fallback_y)
    normal = _unit(_point3_or_none(ref.get("normal")) or fallback_normal, fallback=fallback_normal)
    return origin, x_axis, y_axis, normal


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
    plane = _frame_plane(center, x_axis, y_axis, z_axis)
    box = rg.Box(
        plane,
        rg.Interval(-0.5 * lx, 0.5 * lx),
        rg.Interval(-0.5 * ly, 0.5 * ly),
        rg.Interval(-0.5 * lz, 0.5 * lz),
    )
    return box.ToBrep()


def _frame_plane(center: Point3, x_axis: Point3, y_axis: Point3, z_axis: Point3):
    """Build a right-handed plane that respects the requested local z axis."""
    x = _unit(x_axis, fallback=(1.0, 0.0, 0.0))
    y = _unit(y_axis, fallback=(0.0, 1.0, 0.0))
    z = _unit(z_axis, fallback=(0.0, 0.0, 1.0))
    cross_xy = _cross(x, y)
    if cross_xy[0] * z[0] + cross_xy[1] * z[1] + cross_xy[2] * z[2] < 0.0:
        y = _scale(y, -1.0)
    return rg.Plane(
        rg.Point3d(center[0], center[1], center[2]),
        rg.Vector3d(x[0], x[1], x[2]),
        rg.Vector3d(y[0], y[1], y[2]),
    )


def _brep_box_payload(brep) -> Dict[str, object]:
    if rg is None or brep is None:
        return {}
    try:
        box = brep.GetBoundingBox(True)
        return {
            "min": (box.Min.X, box.Min.Y, box.Min.Z),
            "max": (box.Max.X, box.Max.Y, box.Max.Z),
        }
    except Exception:
        return {}


def _bbox_overlap(a: Dict[str, object], b: Dict[str, object], tolerance: float) -> Optional[bool]:
    amin = a.get("min")
    amax = a.get("max")
    bmin = b.get("min")
    bmax = b.get("max")
    if not (
        isinstance(amin, Sequence)
        and isinstance(amax, Sequence)
        and isinstance(bmin, Sequence)
        and isinstance(bmax, Sequence)
    ):
        return None
    return all(
        float(amax[i]) + tolerance >= float(bmin[i])
        and float(bmax[i]) + tolerance >= float(amin[i])
        for i in range(3)
    )


def _boolean_difference_with_fallback(part, cutter, tolerance: float):
    """Try Rhino's direct overload first, then the Brep[] overload."""
    diff = rg.Brep.CreateBooleanDifference(part, cutter, tolerance)
    if diff and len(diff) > 0:
        return diff, "single"
    try:
        from System import Array  # type: ignore

        target_array = Array[rg.Brep]([part])
        cutter_array = Array[rg.Brep]([cutter])
        diff = rg.Brep.CreateBooleanDifference(target_array, cutter_array, tolerance)
        if diff and len(diff) > 0:
            return diff, "array"
    except Exception:
        pass
    return None, "none"


def _cap_planar_brep_if_needed(brep, tolerance: float):
    if brep is None or getattr(brep, "IsSolid", False):
        return brep, False
    try:
        capped = brep.CapPlanarHoles(tolerance)
        if capped is not None:
            return capped, bool(getattr(capped, "IsSolid", False))
    except Exception:
        pass
    return brep, False


def _slot_intersection_diagnostics(part, cutter, tolerance: float) -> Dict[str, object]:
    info: Dict[str, object] = {
        "target_valid": getattr(part, "IsValid", None),
        "target_solid": getattr(part, "IsSolid", None),
        "cutter_valid": getattr(cutter, "IsValid", None),
        "cutter_solid": getattr(cutter, "IsSolid", None),
    }
    target_box = _brep_box_payload(part)
    cutter_box = _brep_box_payload(cutter)
    info["target_bbox"] = target_box
    info["cutter_bbox"] = cutter_box
    info["bbox_overlap"] = _bbox_overlap(target_box, cutter_box, tolerance)
    try:
        intersection = rg.Brep.CreateBooleanIntersection(part, cutter, tolerance)
        info["boolean_intersection_count"] = len(intersection) if intersection else 0
    except Exception as exc:
        info["boolean_intersection_error"] = str(exc)
    try:
        brep_brep = rg.Intersect.Intersection.BrepBrep(part, cutter, tolerance)
        if isinstance(brep_brep, tuple) and len(brep_brep) >= 3:
            success, curves, points = brep_brep[0], brep_brep[1], brep_brep[2]
            info["brep_brep_success"] = bool(success)
            info["brep_brep_curve_count"] = len(curves) if curves else 0
            info["brep_brep_point_count"] = len(points) if points else 0
    except Exception as exc:
        info["brep_brep_error"] = str(exc)
    return info


def _cylinder_brep(start: Point3, end: Point3, radius: float):
    if rg is None:
        return None
    p0 = rg.Point3d(start[0], start[1], start[2])
    p1 = rg.Point3d(end[0], end[1], end[2])
    axis = p1 - p0
    length = axis.Length
    if length <= 1e-9 or radius <= 1e-9:
        return None
    # Rhino cylinders extrude from the circle plane origin in the positive
    # normal direction. Use the requested start point as the base plane so the
    # resulting cutter spans exactly from start to end.
    plane = rg.Plane(p0, axis)
    cyl = rg.Cylinder(rg.Circle(plane, radius), length)
    return cyl.ToBrep(True, True)


def _plane_cutter_brep(
    origin: Point3,
    x_axis: Point3,
    y_axis: Point3,
    span: float,
):
    if rg is None or span <= 0.0:
        return None
    plane = rg.Plane(
        rg.Point3d(origin[0], origin[1], origin[2]),
        rg.Vector3d(x_axis[0], x_axis[1], x_axis[2]),
        rg.Vector3d(y_axis[0], y_axis[1], y_axis[2]),
    )
    surface = rg.PlaneSurface(
        plane,
        rg.Interval(-span, span),
        rg.Interval(-span, span),
    )
    return surface.ToBrep()


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


def _plate_frame_from_member_dict(
    member: Dict[str, object],
    bottom_face_mode: str,
) -> Tuple[Point3, Point3, Point3]:
    """
    Compute (normal, x_axis, y_axis) for the base plate at an individual member,
    mirroring base_plate_geometry._plane_axes_from_member() but working from a dict.

    * "Perpendicular_to_grain": plate face ⊥ grain → normal = grain direction.
    * "Parallel_to_ground": plate face is horizontal → normal = world Z.
    """
    direction_raw = member.get("direction")
    start_raw = member.get("start")
    end_raw = member.get("end")

    if isinstance(direction_raw, Sequence) and not isinstance(direction_raw, (str, bytes)) and len(direction_raw) >= 3:
        axis = _unit(_to_point3(direction_raw), fallback=(0.0, 0.0, 1.0))
    elif (
        isinstance(start_raw, Sequence) and not isinstance(start_raw, (str, bytes)) and len(start_raw) >= 3
        and isinstance(end_raw, Sequence) and not isinstance(end_raw, (str, bytes)) and len(end_raw) >= 3
    ):
        axis = _unit(_vector(_to_point3(start_raw), _to_point3(end_raw)), fallback=(0.0, 0.0, 1.0))
    else:
        axis = (0.0, 0.0, 1.0)

    mode_normalized = str(bottom_face_mode or "").strip().lower().replace(" ", "_").replace("-", "_")
    if mode_normalized in ("parallel_to_ground", "paralleltoground", "horizontal"):
        normal: Point3 = (0.0, 0.0, 1.0)
        xy_len = (axis[0] ** 2 + axis[1] ** 2) ** 0.5
        x_axis: Point3 = _unit((axis[0], axis[1], 0.0), fallback=(1.0, 0.0, 0.0)) if xy_len > 1e-9 else (1.0, 0.0, 0.0)
        y_axis: Point3 = _unit(_cross(normal, x_axis), fallback=(0.0, 1.0, 0.0))
    else:
        # Perpendicular_to_grain (default)
        normal = axis
        ref: Point3 = (0.0, 0.0, 1.0)
        y_axis = _cross(normal, ref)
        if abs(y_axis[0]) + abs(y_axis[1]) + abs(y_axis[2]) <= 1e-9:
            ref = (1.0, 0.0, 0.0)
            y_axis = _cross(normal, ref)
        y_axis = _unit(y_axis, fallback=(0.0, 1.0, 0.0))
        x_axis = _unit(_cross(y_axis, normal), fallback=(1.0, 0.0, 0.0))

    return normal, x_axis, y_axis


def _member_base_and_top(
    member: Dict[str, object],
    source_units: str,
    output_units: str,
) -> Tuple[Point3, Point3]:
    """Return (base_end_mm, top_end_mm) where base = lower-Z endpoint."""
    start_raw = member.get("start")
    end_raw = member.get("end")
    s = _to_point3(start_raw) if isinstance(start_raw, Sequence) and not isinstance(start_raw, (str, bytes)) and len(start_raw) >= 3 else (0.0, 0.0, 0.0)
    e = _to_point3(end_raw) if isinstance(end_raw, Sequence) and not isinstance(end_raw, (str, bytes)) and len(end_raw) >= 3 else (0.0, 0.0, 0.0)
    base_src, top_src = (s, e) if s[2] <= e[2] else (e, s)
    return (
        _scale_point_units(base_src, source_units, output_units),
        _scale_point_units(top_src, source_units, output_units),
    )


def build_ct_records(
    geometry_payload: Dict[str, object],
    calc_payload: Dict[str, object],
    bottom_face_mode: str,
    process_only_passed: bool = False,
) -> List[Dict[str, object]]:
    """
    Build one CT record per individual timber column (16 total for 4 clusters of 4).

    Each cluster's adjusted_base_plates entry (one per cluster representative) is
    expanded to one record per cluster member using support_cluster_member_indices
    from the geometry payload metadata.  Per-member geometry (start/end, frame) is
    used so that hole locations track each individual timber, not just the cluster
    centroid.

    Hole diameters include the galvanized-layer clearance (hole_clearance from
    fabrication_parameters).  Flat-bottom counterbores (washer/nut recess) are
    emitted on both timber faces as additional entries in the holes list with
    type "flat_bottom_counterbore".  Slot width is calculated from checked
    plate thickness plus the configured side clearances.

    Timber bottom face position reflects the adjustment_along_member shift that was
    applied during the clearance-gap check, exposed via timber.bottom_face_adjusted.
    """
    source_geometry_units = _geometry_payload_units(geometry_payload)
    output_units = "millimeters"
    member_results = _member_result_map(calc_payload)
    members = _member_map(geometry_payload)
    footing_records = _footing_record_map(geometry_payload)
    engineering = calc_payload.get("engineering") if isinstance(calc_payload.get("engineering"), dict) else {}
    sizing = engineering.get("sizing_recommendations") if isinstance(engineering, dict) and isinstance(engineering.get("sizing_recommendations"), dict) else {}
    fabrication = engineering.get("fabrication_parameters") if isinstance(engineering, dict) and isinstance(engineering.get("fabrication_parameters"), dict) else {}
    geometry_handoff = geometry_payload.get("handoff") if isinstance(geometry_payload.get("handoff"), dict) else {}
    geometry_milling = geometry_handoff.get("milling") if isinstance(geometry_handoff, dict) and isinstance(geometry_handoff.get("milling"), dict) else {}

    def _geometry_milling_mm(key: str, default: float = 0.0) -> float:
        value = geometry_milling.get(key)
        if value is None:
            return default
        return float(_scale_optional_dimension(value, source_geometry_units, output_units))

    # ── Cluster-to-individual member index map ───────────────────────────────
    metadata = geometry_payload.get("metadata") or {}
    support_cluster_member_indices: Dict[str, List[int]] = {}
    if isinstance(metadata, dict):
        raw = metadata.get("support_cluster_member_indices")
        if isinstance(raw, dict):
            support_cluster_member_indices = {str(k): [int(i) for i in v] for k, v in raw.items() if isinstance(v, (list, tuple))}

    members_by_index: Dict[int, Dict[str, object]] = {}
    for m in geometry_payload.get("members", []):
        if isinstance(m, dict):
            idx = _member_index(m)
            if idx is not None:
                members_by_index[idx] = m

    # ── Shared sizing / fabrication parameters (cluster-level) ───────────────
    rows = max(int(sizing.get("rows") or geometry_milling.get("plate_hole_rows") or 2), 1)
    holes_per_row = max(int(sizing.get("holes_per_row") or geometry_milling.get("plate_holes_per_row") or 2), 1)
    pitch_parallel = float(sizing.get("pitch_parallel") or _geometry_milling_mm("pitch_parallel", 100.0))
    gage_perp = float(sizing.get("gage_perp") or _geometry_milling_mm("gage_perp", 60.0))
    nominal_hole_dia = float(sizing.get("bolt_hole_dia") or _geometry_milling_mm("bolt_hole_diameter", 0.0))
    slot_clearance_each_side_mm = float(fabrication.get("slot_clearance_each_side") or 0.0)
    slot_extra_length_mm = float(fabrication.get("slot_extra_length") or slot_clearance_each_side_mm)
    slot_extra_depth_mm = float(fabrication.get("slot_extra_depth") or 0.0)

    # Galvanized layer tolerance: bore holes slightly larger than nominal so the
    # bolt still fits after the steel plate has been hot-dip galvanized.
    hole_clearance_mm = float(fabrication.get("hole_clearance") or 0.0)
    effective_hole_dia = nominal_hole_dia + hole_clearance_mm

    # Washer / nut counterbore at each face of the timber (flat-bottom recess).
    washer_recess_depth_mm = float(fabrication.get("washer_recess_depth") or 0.0)
    bolt_dia_mm = float(sizing.get("bolt_dia") or 0.0)
    washer_face_dia_factor = float(
        fabrication.get("washer_face_dia_factor")
        or sizing.get("washer_face_dia_factor")
        or 3.75
    )
    washer_face_dia_auto_mm = float(
        fabrication.get("washer_face_dia_auto")
        or sizing.get("washer_face_dia_auto")
        or (washer_face_dia_factor * bolt_dia_mm if bolt_dia_mm > 0.0 else 0.0)
    )
    provided_washer_face_dia_mm = float(fabrication.get("washer_face_dia") or 0.0)
    if provided_washer_face_dia_mm > 0.0:
        washer_face_dia_mm = provided_washer_face_dia_mm
        washer_face_dia_source = "fabrication_payload"
    else:
        washer_face_dia_mm = washer_face_dia_auto_mm
        washer_face_dia_source = "auto_from_bolt_dia" if washer_face_dia_auto_mm > 0.0 else "missing"
    washer_face_dia_matches_auto = (
        abs(washer_face_dia_mm - washer_face_dia_auto_mm) <= 1e-6
        if washer_face_dia_auto_mm > 0.0
        else None
    )

    records: List[Dict[str, object]] = []

    for plate in calc_payload.get("adjusted_base_plates", []):
        if not isinstance(plate, dict):
            continue

        representative_id = str(plate.get("member_id"))
        rep_result = member_results.get(representative_id, {})
        rep_passed = bool(rep_result.get("passed", False))
        footing_record = footing_records.get(representative_id, {})
        footing_metadata = footing_record.get("metadata") if isinstance(footing_record, dict) else {}
        footing_metadata = footing_metadata if isinstance(footing_metadata, dict) else {}
        footing_geometry = footing_metadata.get("geometry") if isinstance(footing_metadata.get("geometry"), dict) else {}
        footing_milling = footing_metadata.get("milling") if isinstance(footing_metadata.get("milling"), dict) else {}
        incident_member_indices = footing_record.get("incident_member_indices") if isinstance(footing_record, dict) else []
        incident_member_indices = [int(value) for value in incident_member_indices] if isinstance(incident_member_indices, (list, tuple)) else []

        if process_only_passed and not rep_passed:
            continue

        # ── Adjustment along member (same for all cluster members) ───────────
        # adjustment_along_member is already in source_geometry_units because
        # run_validation() scales it back before writing adjusted_base_plates.
        adjustment_source = float(plate.get("adjustment_along_member") or 0.0)
        adjustment_mm = float(_scale_optional_dimension(adjustment_source, source_geometry_units, output_units))

        # ── Expand representative → individual cluster members ───────────────
        cluster_indices = support_cluster_member_indices.get(representative_id, [])
        if not cluster_indices:
            cluster_indices = incident_member_indices
        individual_members: List[Dict[str, object]] = [
            members_by_index[idx] for idx in cluster_indices if idx in members_by_index
        ]
        incident_members = footing_record.get("incident_members") if isinstance(footing_record, dict) else []
        if isinstance(incident_members, list):
            handoff_members = [item for item in incident_members if isinstance(item, dict)]
            if handoff_members:
                individual_members = handoff_members
        if not individual_members:
            # No cluster data: fall back to the representative member only.
            rep_member = members.get(representative_id)
            if rep_member:
                individual_members = [rep_member]
        if not individual_members:
            continue

        for fallback_index, indiv_member in enumerate(individual_members):
            indiv_id = str(indiv_member.get("member_id", representative_id))
            indiv_result = member_results.get(indiv_id) or rep_result
            indiv_passed = bool(indiv_result.get("passed", rep_passed))
            indiv_index = _member_index(indiv_member)
            if indiv_index is not None and indiv_index in incident_member_indices:
                local_plate_index = incident_member_indices.index(indiv_index)
            else:
                local_plate_index = fallback_index

            plate_ref = _item_for_member_index(
                footing_geometry.get("plate_specs"),
                indiv_index,
                local_plate_index,
            )
            plate_frame_ref = _item_for_member_index(
                footing_geometry.get("plate_frames"),
                indiv_index,
                local_plate_index,
            )
            bottom_face_ref = _item_for_member_index(
                footing_geometry.get("timber_bottom_face_refs"),
                indiv_index,
                local_plate_index,
            )
            plate_hole_ref = _list_item(footing_milling.get("plate_hole_specs"), local_plate_index)
            plate_hole_centers_ref = _list_item(
                footing_milling.get("plate_hole_centers"),
                local_plate_index,
            )

            if process_only_passed and not indiv_passed:
                continue

            # ── Individual member geometry ────────────────────────────────────
            start_raw = indiv_member.get("start")
            end_raw = indiv_member.get("end")
            s_src = _to_point3(start_raw) if isinstance(start_raw, Sequence) and not isinstance(start_raw, (str, bytes)) and len(start_raw) >= 3 else (0.0, 0.0, 0.0)
            e_src = _to_point3(end_raw) if isinstance(end_raw, Sequence) and not isinstance(end_raw, (str, bytes)) and len(end_raw) >= 3 else (0.0, 0.0, 0.0)

            # Base end = lower-Z endpoint (the support end that gets milled)
            base_src, top_src = (s_src, e_src) if s_src[2] <= e_src[2] else (e_src, s_src)
            member_axis = _unit(_vector(base_src, top_src), fallback=(0.0, 0.0, 1.0))

            # The geometry component already resolved the web-plate target for
            # this exact timber member. Prefer that frame over rebuilding one.
            fallback_plate_center_src = _add(base_src, _scale(member_axis, adjustment_source))
            fallback_normal, fallback_x_axis, fallback_y_axis = _plate_frame_from_member_dict(
                indiv_member,
                bottom_face_mode,
            )
            if isinstance(plate_ref, dict):
                fallback_plate_center_src = _point3_or_none(plate_ref.get("center")) or fallback_plate_center_src
            plate_center_src, x_axis, y_axis, normal = _frame_from_ref(
                plate_frame_ref,
                fallback_origin=fallback_plate_center_src,
                fallback_x=fallback_x_axis,
                fallback_y=fallback_y_axis,
                fallback_normal=fallback_normal,
            )
            plate_center_mm = _scale_point_units(plate_center_src, source_geometry_units, output_units)

            # Timber endpoints in output units
            s_mm = _scale_point_units(s_src, source_geometry_units, output_units)
            e_mm = _scale_point_units(e_src, source_geometry_units, output_units)
            base_mm = _scale_point_units(base_src, source_geometry_units, output_units)
            top_mm = _scale_point_units(top_src, source_geometry_units, output_units)

            # The geometry handoff owns the timber cut plane. Use that explicit
            # reference rather than estimating a new trim point from the base end.
            bottom_face_origin_src = None
            bottom_face_plane_ref = bottom_face_ref if isinstance(bottom_face_ref, dict) else {}
            if isinstance(bottom_face_plane_ref, dict):
                bottom_face_origin_src = _point3_or_none(bottom_face_plane_ref.get("origin"))
            if bottom_face_origin_src is None:
                bottom_face_origin_src = _add(base_src, _scale(member_axis, adjustment_source))
            bottom_face_adjusted_mm = _scale_point_units(
                bottom_face_origin_src,
                source_geometry_units,
                output_units,
            )

            timber_width_mm = float(_scale_optional_dimension(indiv_member.get("width") or 0.0, source_geometry_units, output_units))
            timber_height_mm = float(_scale_optional_dimension(indiv_member.get("height") or 0.0, source_geometry_units, output_units))
            timber_length_mm = float(_scale_optional_dimension(indiv_member.get("length") or 0.0, source_geometry_units, output_units))
            timber_center_mm = _center_of_points([s_mm, e_mm])

            # The webplate handoff already resolves the extra gap from the
            # baseplate heel to the timber bottom face. The timber slot starts
            # at that bottom face and follows the in-timber plate engagement
            # length, which is the checked code pattern span plus the small
            # fabrication clearance above the plate tip.
            webplate_effective_length_mm = float(
                (
                    _scale_optional_dimension(
                        plate_ref.get("effective_plate_length")
                        or plate_ref.get("required_heel_to_tip_distance")
                        or plate_ref.get("length"),
                        source_geometry_units,
                        output_units,
                    )
                    if isinstance(plate_ref, dict)
                    else 0.0
                )
                or 0.0
            )
            plate_pattern_length_mm = float(
                (
                    _scale_optional_dimension(
                        plate_ref.get("code_pattern_length"),
                        source_geometry_units,
                        output_units,
                    )
                    if isinstance(plate_ref, dict) and plate_ref.get("code_pattern_length") is not None
                    else None
                )
                or sizing.get("plate_length")
                or 0.0
            )
            plate_width_mm = float(
                sizing.get("plate_width")
                or (
                    _scale_optional_dimension(plate_ref.get("width"), source_geometry_units, output_units)
                    if isinstance(plate_ref, dict)
                    else 0.0
                )
                or 0.0
            )
            plate_thickness_mm = float(
                sizing.get("plate_thickness")
                or (
                    _scale_optional_dimension(plate_ref.get("thickness"), source_geometry_units, output_units)
                    if isinstance(plate_ref, dict)
                    else 0.0
                )
                or 0.0
            )
            slot_length = plate_pattern_length_mm + slot_extra_length_mm
            slot_width = plate_thickness_mm + 2.0 * slot_clearance_each_side_mm
            slot_depth = min(timber_height_mm, plate_width_mm + slot_extra_depth_mm)
            slot_start = bottom_face_adjusted_mm
            slot_end = _add(slot_start, _scale(x_axis, slot_length))
            # Once the trimmed timber Brep is capped back to a solid, the slot
            # cutter only needs a small entry overrun to clear the face.
            slot_entry_overshoot_mm = max(2.0, _boolean_tolerance(output_units) * 10.0)
            # Keep the design slot depth exact, but let the Boolean cutter run
            # well past the timber depth faces. The earlier 1.5 mm overrun was
            # enough for fabrication intent, yet still too close to a tangent
            # condition for RhinoCommon on these skewed members.
            slot_depth_boolean_overshoot_each_side_mm = max(
                10.0,
                slot_clearance_each_side_mm,
                _boolean_tolerance(output_units) * 100.0,
            )
            slot_cutter_start = _add(slot_start, _scale(x_axis, -slot_entry_overshoot_mm))
            slot_cutter_end = slot_end
            slot_cutter_length = slot_length + slot_entry_overshoot_mm
            slot_cutter_depth = slot_depth + 2.0 * slot_depth_boolean_overshoot_each_side_mm
            slot_center_axis = _center_of_points([slot_cutter_start, slot_cutter_end])
            # The webplate centerline and timber centerline are shared by
            # contract. Recenter the cutter through the timber depth so the
            # through-slot clears both depth faces symmetrically even when the
            # bottom-face reference point lies on an offset collision plane.
            slot_axis_from_timber_center = _vector(timber_center_mm, slot_center_axis)
            slot_center_depth_offset = (
                slot_axis_from_timber_center[0] * y_axis[0]
                + slot_axis_from_timber_center[1] * y_axis[1]
                + slot_axis_from_timber_center[2] * y_axis[2]
            )
            slot_depth_recenter_vector = _scale(y_axis, -slot_center_depth_offset)
            slot_start = _add(slot_start, slot_depth_recenter_vector)
            slot_end = _add(slot_end, slot_depth_recenter_vector)
            slot_cutter_start = _add(slot_cutter_start, slot_depth_recenter_vector)
            slot_cutter_end = _add(slot_cutter_end, slot_depth_recenter_vector)
            slot_center = _add(slot_center_axis, slot_depth_recenter_vector)

            # Align the timber axes to the web-plate frame used upstream:
            # length follows the webplate axis; width follows bolt direction;
            # depth follows the webplate depth axis.
            timber_x = x_axis
            timber_y = normal
            timber_z = y_axis

            # Hole and recess cutters are referenced from the true timber side
            # faces. The prior max(height, depth) span could start recesses far
            # outside the member and leave only a tiny overlap to subtract.
            cutter_overshoot_mm = max(2.0, _boolean_tolerance(output_units) * 10.0)
            half_timber_width_mm = 0.5 * timber_width_mm
            hole_depth = timber_width_mm + 2.0 * cutter_overshoot_mm

            # The hole locations already exist in the geometry handoff. Reuse
            # them exactly, and fall back to the older grid only when a legacy
            # payload does not provide per-plate centers.
            hole_centers = []
            if isinstance(plate_hole_centers_ref, (list, tuple)):
                for center_ref in plate_hole_centers_ref:
                    center_src = _point3_or_none(center_ref)
                    if center_src is not None:
                        hole_centers.append(
                            _scale_point_units(center_src, source_geometry_units, output_units)
                        )
            if not hole_centers:
                local_rows = rows
                local_holes_per_row = holes_per_row
                local_pitch = pitch_parallel
                local_gage = gage_perp
                if isinstance(plate_hole_ref, dict):
                    local_rows = max(int(plate_hole_ref.get("row_count") or rows), 1)
                    local_holes_per_row = max(int(plate_hole_ref.get("holes_per_row") or holes_per_row), 1)
                    local_pitch = float(_scale_optional_dimension(plate_hole_ref.get("pitch") or 0.0, source_geometry_units, output_units) or pitch_parallel)
                    local_gage = float(_scale_optional_dimension(plate_hole_ref.get("row_spacing") or 0.0, source_geometry_units, output_units) or gage_perp)
                hole_centers = _hole_grid_centers(
                    center=plate_center_mm,
                    x_axis=x_axis,
                    y_axis=y_axis,
                    rows=local_rows,
                    holes_per_row=local_holes_per_row,
                    pitch=local_pitch,
                    gage=local_gage,
                )

            holes: List[Dict[str, object]] = []
            counterbores: List[Dict[str, object]] = []
            for idx, hc in enumerate(hole_centers):
                # Through-hole runs along the plate normal (bolt axis)
                near_face_center = _add(hc, _scale(normal, -half_timber_width_mm))
                far_face_center = _add(hc, _scale(normal, half_timber_width_mm))
                hs = _add(near_face_center, _scale(normal, -cutter_overshoot_mm))
                he = _add(far_face_center, _scale(normal, cutter_overshoot_mm))

                # Through-hole: nominal diameter + galvanized clearance
                holes.append(
                    {
                        "index": idx,
                        "type": "through_hole",
                        "center": hc,
                        "axis_start": hs,
                        "axis_end": he,
                        "diameter": effective_hole_dia,
                        "nominal_diameter": nominal_hole_dia,
                        "clearance_added": hole_clearance_mm,
                        "depth": hole_depth,
                    }
                )

                # Counterbore at entry face (near side)
                if washer_recess_depth_mm > 0.0 and washer_face_dia_mm > 0.0:
                    cb_near_end = _add(near_face_center, _scale(normal, washer_recess_depth_mm))
                    holes.append(
                        {
                            "index": idx,
                            "type": "flat_bottom_counterbore",
                            "operation": "washer_recess",
                            "face": "near",
                            "center": near_face_center,
                            "axis_start": hs,
                            "axis_end": cb_near_end,
                            "diameter": washer_face_dia_mm,
                            "depth": washer_recess_depth_mm,
                            "bolt_hole_index": idx,
                        }
                    )
                    counterbores.append(holes[-1])
                    # Counterbore at exit face (far side)
                    cb_far_start = _add(far_face_center, _scale(normal, -washer_recess_depth_mm))
                    holes.append(
                        {
                            "index": idx,
                            "type": "flat_bottom_counterbore",
                            "operation": "washer_recess",
                            "face": "far",
                            "center": far_face_center,
                            "axis_start": cb_far_start,
                            "axis_end": he,
                            "diameter": washer_face_dia_mm,
                            "depth": washer_recess_depth_mm,
                            "bolt_hole_index": idx,
                        }
                    )
                    counterbores.append(holes[-1])

            slot = {
                "type": "slot_cut",
                "center": slot_center,
                "axis_center_before_depth_recenter": slot_center_axis,
                "start": slot_start,
                "end": slot_end,
                "cutter_start": slot_cutter_start,
                "cutter_end": slot_cutter_end,
                "x_axis": x_axis,
                "depth_axis": y_axis,
                "width_axis": normal,
                "y_axis": y_axis,
                "normal": normal,
                "length": slot_cutter_length,
                "design_length": slot_length,
                "entry_overshoot": slot_entry_overshoot_mm,
                "width": slot_width,
                "depth": slot_depth,
                "cutter_depth": slot_cutter_depth,
                "depth_overshoot_each_side": slot_depth_boolean_overshoot_each_side_mm,
                "design_depth": slot_depth,
                "boolean_depth_overshoot_each_side": slot_depth_boolean_overshoot_each_side_mm,
                "depth_recenter_offset": -slot_center_depth_offset,
                "webplate_effective_length": webplate_effective_length_mm,
                "plate_pattern_length": plate_pattern_length_mm,
                "plate_width": plate_width_mm,
                "plate_thickness": plate_thickness_mm,
                "slot_clearance_each_side": slot_clearance_each_side_mm,
                "slot_extra_length": slot_extra_length_mm,
                "slot_extra_depth": slot_extra_depth_mm,
                "length_source": "geometry_code_pattern_length_plus_slot_top_clearance",
                "start_source": "timber_bottom_face_plane",
                "cutter_start_source": "timber_bottom_face_plane_minus_entry_overshoot",
                "dimension_source": "resolved_hole_pattern_plus_tolerances",
            }

            record = {
                "member_id": indiv_id,
                "representative_member_id": representative_id,
                "member_index": int(indiv_index if indiv_index is not None else plate.get("member_index") or 0),
                "geometry_units": output_units,
                "source_geometry_units": source_geometry_units,
                "preview_units": source_geometry_units,
                "group": indiv_member.get("group") or plate.get("group"),
                "level": indiv_member.get("level") or members.get(representative_id, {}).get("level"),
                "status": "PASS" if indiv_passed else "FAIL",
                "bottom_face_mode": bottom_face_mode,
                "adjustment_along_member": adjustment_mm,
                "timber": {
                    "center": timber_center_mm,
                    "x_axis": timber_x,
                    "y_axis": timber_y,
                    "z_axis": timber_z,
                    "start": s_mm,
                    "end": e_mm,
                    # Base end and adjusted bottom cut face (CT milling reference)
                    "base_end": base_mm,
                    "top_end": top_mm,
                    "bottom_face_adjusted": bottom_face_adjusted_mm,
                    "bottom_face_plane": {
                        "origin": bottom_face_adjusted_mm,
                        "x_axis": bottom_face_plane_ref.get("x_axis") if isinstance(bottom_face_plane_ref, dict) else None,
                        "y_axis": bottom_face_plane_ref.get("y_axis") if isinstance(bottom_face_plane_ref, dict) else None,
                        "normal": bottom_face_plane_ref.get("normal") if isinstance(bottom_face_plane_ref, dict) else None,
                        "origin_source": bottom_face_plane_ref.get("origin_source") if isinstance(bottom_face_plane_ref, dict) else None,
                    },
                    "length": timber_length_mm,
                    "width": timber_width_mm,
                    "height": timber_height_mm,
                },
                "milling_geometry": {
                    "holes": holes,
                    "counterbores": counterbores,
                    "washer_recesses": counterbores,
                    "slot": slot,
                },
                "source_references": {
                    "plate_ref_source": "geometry_handoff" if isinstance(plate_ref, dict) else "fallback",
                    "plate_frame_ref_source": "geometry_handoff" if isinstance(plate_frame_ref, dict) else "fallback",
                    "bottom_face_ref_source": "geometry_handoff" if isinstance(bottom_face_ref, dict) else "fallback",
                    "hole_center_ref_source": "geometry_handoff" if hole_centers and isinstance(plate_hole_centers_ref, (list, tuple)) else "fallback",
                    "footing_member_id": representative_id,
                    "local_plate_index": local_plate_index,
                },
                "validation": indiv_result,
                "milling": {
                    "anchor_pattern": "concealed_slotted_plate_through_bolts",
                    "clearance_rule_mm": calc_payload.get("metadata", {}).get("min_allowable_clearance")
                    if isinstance(calc_payload.get("metadata"), dict)
                    else None,
                    "nominal_bolt_hole_dia": nominal_hole_dia,
                    "effective_bolt_hole_dia": effective_hole_dia,
                    "hole_clearance_added": hole_clearance_mm,
                    "bolt_dia": bolt_dia_mm,
                    "counterbore_dia": washer_face_dia_mm,
                    "counterbore_dia_auto": washer_face_dia_auto_mm,
                    "counterbore_dia_factor": washer_face_dia_factor,
                    "counterbore_dia_source": washer_face_dia_source,
                    "counterbore_matches_auto": washer_face_dia_matches_auto,
                    "counterbore_depth": washer_recess_depth_mm,
                    "counterbores_per_through_hole": 2 if washer_recess_depth_mm > 0.0 and washer_face_dia_mm > 0.0 else 0,
                    "pitch_parallel": pitch_parallel,
                    "gage_perp": gage_perp,
                    "end_distance": sizing.get("end_distance") if isinstance(sizing, dict) else None,
                    "edge_distance": sizing.get("edge_distance") if isinstance(sizing, dict) else None,
                    "tolerances": {
                        "hole_clearance": hole_clearance_mm,
                        "slot_clearance_each_side": slot_clearance_each_side_mm,
                        "slot_extra_length": slot_extra_length_mm,
                        "slot_extra_depth": slot_extra_depth_mm,
                        "washer_recess_depth": washer_recess_depth_mm,
                        "washer_face_dia": washer_face_dia_mm,
                        "washer_face_dia_auto": washer_face_dia_auto_mm,
                        "washer_face_dia_factor": washer_face_dia_factor,
                        "washer_face_dia_source": washer_face_dia_source,
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
        geometry_units = _canonical_units(rec.get("geometry_units"), "millimeters")
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

        c = _scale_point_units(
            (float(center[0]), float(center[1]), float(center[2])),
            geometry_units,
            "meters",
        )
        x = _unit((float(x_axis[0]), float(x_axis[1]), float(x_axis[2])), fallback=(1.0, 0.0, 0.0))
        y = _unit((float(y_axis[0]), float(y_axis[1]), float(y_axis[2])), fallback=_safe_y_axis(x))

        width = float(_scale_optional_dimension(timber.get("width") or 0.0, geometry_units, "meters"))
        height = float(_scale_optional_dimension(timber.get("height") or 0.0, geometry_units, "meters"))
        length = float(_scale_optional_dimension(timber.get("length") or 0.0, geometry_units, "meters"))

        features: List[Dict[str, object]] = []
        for hole in holes if isinstance(holes, list) else []:
            if not isinstance(hole, dict):
                continue
            hole_type = str(hole.get("type") or "through_hole")
            if hole_type == "flat_bottom_counterbore":
                dtype = "ct.milling/Counterbore"
            else:
                dtype = "ct.milling/Hole"
            features.append(
                {
                    "dtype": dtype,
                    "data": {
                        "hole_type": hole_type,
                        "face": hole.get("face"),
                        "bolt_hole_index": hole.get("bolt_hole_index"),
                        "center": _scale_optional_point(hole.get("center"), geometry_units, "meters"),
                        "axis_start": _scale_optional_point(hole.get("axis_start"), geometry_units, "meters"),
                        "axis_end": _scale_optional_point(hole.get("axis_end"), geometry_units, "meters"),
                        "diameter": _scale_optional_dimension(hole.get("diameter"), geometry_units, "meters"),
                        "depth": _scale_optional_dimension(hole.get("depth"), geometry_units, "meters"),
                    },
                }
            )
        if isinstance(slot, dict):
            features.append(
                {
                    "dtype": "ct.milling/Slot",
                    "data": {
                        "center": _scale_optional_point(slot.get("center"), geometry_units, "meters"),
                        "start": _scale_optional_point(slot.get("start"), geometry_units, "meters"),
                        "end": _scale_optional_point(slot.get("end"), geometry_units, "meters"),
                        "x_axis": slot.get("x_axis"),
                        "width_axis": slot.get("width_axis"),
                        "depth_axis": slot.get("depth_axis"),
                        "y_axis": slot.get("y_axis"),
                        "normal": slot.get("normal"),
                        "length": _scale_optional_dimension(slot.get("length"), geometry_units, "meters"),
                        "width": _scale_optional_dimension(slot.get("width"), geometry_units, "meters"),
                        "depth": _scale_optional_dimension(slot.get("depth"), geometry_units, "meters"),
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


def build_inspection_breps(
    ct_records: Sequence[Dict[str, object]],
    target_units: Optional[str] = None,
    include_cutters: bool = False,
    diagnostics: Optional[List[Dict[str, object]]] = None,
) -> List[object]:
    if rg is None:
        return []

    inspection_breps: List[object] = []

    for rec in ct_records:
        timber = rec.get("timber", {})
        milling_geometry = rec.get("milling_geometry", {})
        if not isinstance(timber, dict) or not isinstance(milling_geometry, dict):
            continue

        record_units = _canonical_units(rec.get("geometry_units"), "millimeters")
        preview_units = _canonical_units(
            target_units or rec.get("preview_units") or rec.get("source_geometry_units"),
            record_units,
        )
        boolean_tolerance = _boolean_tolerance(preview_units)

        center = _point3_or_none(_scale_optional_point(timber.get("center"), record_units, preview_units))
        x_axis = _point3_or_none(timber.get("x_axis"))
        y_axis = _point3_or_none(timber.get("y_axis"))
        z_axis = _point3_or_none(timber.get("z_axis"))
        length = float(_scale_optional_dimension(timber.get("length") or 0.0, record_units, preview_units))
        width = float(_scale_optional_dimension(timber.get("width") or 0.0, record_units, preview_units))
        height = float(_scale_optional_dimension(timber.get("height") or 0.0, record_units, preview_units))

        base_end = _point3_or_none(_scale_optional_point(timber.get("base_end"), record_units, preview_units))
        top_end = _point3_or_none(_scale_optional_point(timber.get("top_end"), record_units, preview_units))
        if center is None:
            start = _point3_or_none(_scale_optional_point(timber.get("start"), record_units, preview_units))
            end = _point3_or_none(_scale_optional_point(timber.get("end"), record_units, preview_units))
            if base_end is not None and top_end is not None:
                center = _center_of_points((base_end, top_end))
            elif start is not None and end is not None:
                center = _center_of_points((start, end))

        if x_axis is None and base_end is not None and top_end is not None:
            x_axis = _vector(base_end, top_end)

        if x_axis is None:
            x_axis = (1.0, 0.0, 0.0)
        x_axis_u = _unit(_to_point3(x_axis), fallback=(1.0, 0.0, 0.0))

        z_seed = _unit(_to_point3(z_axis), fallback=(0.0, 0.0, 1.0)) if z_axis is not None else (0.0, 0.0, 1.0)
        x_dot_z = x_axis_u[0] * z_seed[0] + x_axis_u[1] * z_seed[1] + x_axis_u[2] * z_seed[2]
        if abs(x_dot_z) >= 0.95:
            z_seed = (0.0, 1.0, 0.0)

        y_seed = _cross(z_seed, x_axis_u)
        if y_axis is not None:
            y_seed = _to_point3(y_axis)
        y_axis_u = _unit(y_seed, fallback=(0.0, 1.0, 0.0))
        z_axis_u = _unit(_cross(x_axis_u, y_axis_u), fallback=(0.0, 0.0, 1.0))

        if center is None or length <= 1e-9 or width <= 1e-9 or height <= 1e-9:
            if diagnostics is not None:
                diagnostics.append(
                    {
                        "member_id": rec.get("member_id"),
                        "member_index": rec.get("member_index"),
                        "preview_skip_reason": "missing_frame_or_dimensions",
                        "has_center": center is not None,
                        "length": length,
                        "width": width,
                        "height": height,
                    }
                )
            continue

        timber_brep = _box_brep(
            center=center,
            x_axis=x_axis_u,
            y_axis=y_axis_u,
            z_axis=z_axis_u,
            lx=length,
            ly=width,
            lz=height,
        )
        if timber_brep is None:
            continue

        # Trim the timber by the exact bottom-face plane resolved upstream.
        base_end = base_end
        top_end = top_end
        bottom_face_adjusted = _scale_optional_point(timber.get("bottom_face_adjusted"), record_units, preview_units)
        bottom_face_plane = timber.get("bottom_face_plane") if isinstance(timber.get("bottom_face_plane"), dict) else {}
        plane_trimmed = False
        plane_trim_cap_succeeded = False
        timber_solid_before_trim = bool(getattr(timber_brep, "IsSolid", False))

        plane_origin = _scale_optional_point(bottom_face_plane.get("origin"), record_units, preview_units)
        plane_x = bottom_face_plane.get("x_axis")
        plane_y = bottom_face_plane.get("y_axis")
        if (
            isinstance(plane_origin, Sequence)
            and isinstance(plane_x, Sequence)
            and isinstance(plane_y, Sequence)
            and len(plane_origin) >= 3
            and len(plane_x) >= 3
            and len(plane_y) >= 3
        ):
            span = max(length, width, height) * 4.0
            plane_cutter = _plane_cutter_brep(
                origin=_to_point3(plane_origin),
                x_axis=_unit(_to_point3(plane_x), fallback=(1.0, 0.0, 0.0)),
                y_axis=_unit(_to_point3(plane_y), fallback=(0.0, 1.0, 0.0)),
                span=span,
            )
            if plane_cutter is not None:
                pieces = timber_brep.Split(plane_cutter, boolean_tolerance)
                if pieces and len(pieces) > 0:
                    top_point = (
                        rg.Point3d(*_to_point3(top_end))
                        if isinstance(top_end, Sequence) and len(top_end) >= 3
                        else None
                    )
                    keep = None
                    if top_point is not None:
                        for piece in pieces:
                            try:
                                if piece is not None and piece.IsPointInside(top_point, 0.01, False):
                                    keep = piece
                                    break
                            except Exception:
                                pass
                    if keep is None:
                        member_axis = _unit(
                            _vector(_to_point3(base_end), _to_point3(top_end))
                            if (
                                isinstance(base_end, Sequence)
                                and isinstance(top_end, Sequence)
                                and len(base_end) >= 3
                                and len(top_end) >= 3
                            )
                            else x_axis_u,
                            fallback=x_axis_u,
                        )
                        scored = []
                        for piece in pieces:
                            if piece is None:
                                continue
                            try:
                                center_pt = piece.GetBoundingBox(True).Center
                                score = (
                                    center_pt.X * member_axis[0]
                                    + center_pt.Y * member_axis[1]
                                    + center_pt.Z * member_axis[2]
                                )
                                scored.append((score, piece))
                            except Exception:
                                pass
                        if scored:
                            keep = max(scored, key=lambda item: item[0])[1]
                    if keep is not None:
                        timber_brep, plane_trim_cap_succeeded = _cap_planar_brep_if_needed(
                            keep,
                            boolean_tolerance,
                        )
                        plane_trimmed = True

        if not plane_trimmed and (
            isinstance(base_end, Sequence)
            and isinstance(top_end, Sequence)
            and isinstance(bottom_face_adjusted, Sequence)
            and len(base_end) >= 3
            and len(top_end) >= 3
            and len(bottom_face_adjusted) >= 3
            and length > 1e-6
        ):
            member_axis = _unit(_vector(_to_point3(base_end), _to_point3(top_end)), fallback=_unit(_to_point3(x_axis), fallback=(1.0, 0.0, 0.0)))
            trim_vec = _vector(_to_point3(base_end), _to_point3(bottom_face_adjusted))
            trim_depth = max(0.0, trim_vec[0] * member_axis[0] + trim_vec[1] * member_axis[1] + trim_vec[2] * member_axis[2])
            trim_depth = min(trim_depth, max(length - 1e-6, 0.0))
            if trim_depth > 1e-6:
                x_dot_member = x_axis_u[0] * member_axis[0] + x_axis_u[1] * member_axis[1] + x_axis_u[2] * member_axis[2]
                axis_sign = 1.0 if x_dot_member >= 0.0 else -1.0
                center_shift = _scale(x_axis_u, axis_sign * (0.5 * trim_depth))
                trimmed_center = _add(center, center_shift)
                trimmed_length = max(length - trim_depth, 1e-6)
                trimmed_brep = _box_brep(
                    center=trimmed_center,
                    x_axis=x_axis_u,
                    y_axis=y_axis_u,
                    z_axis=z_axis_u,
                    lx=trimmed_length,
                    ly=width,
                    lz=height,
                )
                if trimmed_brep is not None:
                    timber_brep = trimmed_brep

        timber_solid_after_trim = bool(getattr(timber_brep, "IsSolid", False))

        cutter_specs: List[Dict[str, object]] = []
        holes = milling_geometry.get("holes", []) if isinstance(milling_geometry.get("holes"), list) else []

        for hole in holes:
            if not isinstance(hole, dict):
                continue

            hs = _scale_optional_point(hole.get("axis_start"), record_units, preview_units)
            he = _scale_optional_point(hole.get("axis_end"), record_units, preview_units)
            dia = float(_scale_optional_dimension(hole.get("diameter") or 0.0, record_units, preview_units))

            if isinstance(hs, Sequence) and isinstance(he, Sequence) and len(hs) >= 3 and len(he) >= 3 and dia > 0.0:
                hole_brep = _cylinder_brep(_to_point3(hs), _to_point3(he), 0.5 * dia)
                if hole_brep is not None:
                    cutter_specs.append(
                        {
                            "kind": str(hole.get("type") or "through_hole"),
                            "face": hole.get("face"),
                            "brep": hole_brep,
                        }
                    )

        slot = milling_geometry.get("slot")
        if isinstance(slot, dict):
            sc = _scale_optional_point(slot.get("center"), record_units, preview_units)
            sx = slot.get("x_axis")
            sw_axis = slot.get("width_axis") or slot.get("normal")
            sd_axis = slot.get("depth_axis") or slot.get("y_axis")
            sl = float(_scale_optional_dimension(slot.get("length") or 0.0, record_units, preview_units))
            sw = float(_scale_optional_dimension(slot.get("width") or 0.0, record_units, preview_units))
            sd = float(
                _scale_optional_dimension(
                    slot.get("cutter_depth") or slot.get("depth") or 0.0,
                    record_units,
                    preview_units,
                )
            )
            if (
                isinstance(sc, Sequence)
                and isinstance(sx, Sequence)
                and isinstance(sw_axis, Sequence)
                and isinstance(sd_axis, Sequence)
                and len(sc) >= 3
                and len(sx) >= 3
                and len(sw_axis) >= 3
                and len(sd_axis) >= 3
                and sl > 0.0
                and sw > 0.0
                and sd > 0.0
            ):
                slot_brep = _box_brep(
                    center=_to_point3(sc),
                    x_axis=_unit(_to_point3(sx), fallback=(1.0, 0.0, 0.0)),
                    y_axis=_unit(_to_point3(sw_axis), fallback=(0.0, 1.0, 0.0)),
                    z_axis=_unit(_to_point3(sd_axis), fallback=(0.0, 0.0, 1.0)),
                    lx=sl,
                    ly=sw,
                    lz=sd,
                )
                if slot_brep is not None:
                    cutter_specs.append({"kind": "slot_cut", "face": None, "brep": slot_brep})

        # Cut the primary plate slot before the transverse holes/recesses.
        # RhinoCommon is much more reliable subtracting the long rectangular
        # void from the clean timber solid than trying to push that box through
        # a Brep that already contains intersecting cylindrical openings.
        cutter_specs.sort(
            key=lambda spec: 0 if str(spec.get("kind") or "") == "slot_cut" else 1
        )

        if include_cutters:
            inspection_breps.extend(
                spec["brep"] for spec in cutter_specs if spec.get("brep") is not None
            )

        current_parts = [timber_brep]
        cut_attempts: Dict[str, int] = {}
        cut_successes: Dict[str, int] = {}
        cut_failures: Dict[str, int] = {}
        cut_methods: Dict[str, str] = {}
        slot_target_diagnostics: Dict[str, object] = {}
        deferred_slot_specs: List[Dict[str, object]] = []
        for spec in cutter_specs:
            cutter = spec.get("brep")
            if cutter is None:
                continue
            kind = str(spec.get("kind") or "unknown")
            face = spec.get("face")
            label = "{0}:{1}".format(kind, face) if face else kind
            cut_attempts[label] = cut_attempts.get(label, 0) + 1
            next_parts: List[object] = []
            succeeded = False
            for part in current_parts:
                if kind == "slot_cut" and not slot_target_diagnostics:
                    slot_target_diagnostics = _slot_intersection_diagnostics(
                        part,
                        cutter,
                        boolean_tolerance,
                    )
                diff, cut_method = _boolean_difference_with_fallback(
                    part,
                    cutter,
                    boolean_tolerance,
                )
                if diff and len(diff) > 0:
                    next_parts.extend(item for item in diff if item is not None)
                    succeeded = True
                    cut_methods[label] = cut_method
                else:
                    next_parts.append(part)
            current_parts = next_parts
            if succeeded:
                cut_successes[label] = cut_successes.get(label, 0) + 1
            else:
                cut_failures[label] = cut_failures.get(label, 0) + 1
                if kind == "slot_cut":
                    deferred_slot_specs.append(spec)

        # If Rhino rejects the slot against the clean trimmed timber, try the
        # same slot once more after the through-holes and counterbores have
        # opened additional target faces. This preserves the preferred order
        # when it works, but gives the rectangular cutter a second, materially
        # different Boolean target before giving up.
        for spec in deferred_slot_specs:
            cutter = spec.get("brep")
            if cutter is None:
                continue
            label = "slot_cut:retry_after_holes"
            cut_attempts[label] = cut_attempts.get(label, 0) + 1
            next_parts = []
            succeeded = False
            for part in current_parts:
                diff, cut_method = _boolean_difference_with_fallback(
                    part,
                    cutter,
                    boolean_tolerance,
                )
                if diff and len(diff) > 0:
                    next_parts.extend(item for item in diff if item is not None)
                    succeeded = True
                    cut_methods[label] = cut_method
                else:
                    next_parts.append(part)
            current_parts = next_parts
            if succeeded:
                cut_successes[label] = cut_successes.get(label, 0) + 1
            else:
                cut_failures[label] = cut_failures.get(label, 0) + 1

        inspection_breps.extend(current_parts)
        if diagnostics is not None:
            diagnostics.append(
                {
                    "member_id": rec.get("member_id"),
                    "member_index": rec.get("member_index"),
                    "boolean_tolerance": boolean_tolerance,
                    "timber_solid_before_trim": timber_solid_before_trim,
                    "plane_trimmed": plane_trimmed,
                    "plane_trim_cap_succeeded": plane_trim_cap_succeeded,
                    "timber_solid_after_trim": timber_solid_after_trim,
                    "cutter_count": len(cutter_specs),
                    "cut_attempts": cut_attempts,
                    "cut_successes": cut_successes,
                    "cut_failures": cut_failures,
                    "cut_methods": cut_methods,
                    "slot_target_diagnostics": slot_target_diagnostics,
                    "final_part_count": len(current_parts),
                }
            )

    return inspection_breps


def _effective_geometry_payload(
    geometry_payload: Dict[str, object],
    calc_payload: Dict[str, object],
) -> Tuple[Dict[str, object], bool]:
    # A resolved second-pass geometry payload carries the authoritative footing
    # handoff: plate frames, hole centers, and timber bottom-face references.
    # Prefer it whenever it is explicitly supplied.
    if isinstance(geometry_payload, dict):
        footings = geometry_payload.get("footings")
        handoff = geometry_payload.get("handoff")
        if footings or (isinstance(handoff, dict) and handoff):
            return geometry_payload, False
    maybe_synced = calc_payload.get("synced_geometry_payload")
    if isinstance(maybe_synced, dict) and maybe_synced:
        return maybe_synced, True
    return geometry_payload, False


def _reference_usage_summary(records: Sequence[Dict[str, object]]) -> Dict[str, object]:
    tracked_keys = (
        "plate_ref_source",
        "plate_frame_ref_source",
        "bottom_face_ref_source",
        "hole_center_ref_source",
    )
    source_counts = {
        key: {"geometry_handoff": 0, "fallback": 0}
        for key in tracked_keys
    }
    cluster_counts: Dict[str, int] = {}

    for record in records:
        representative_id = str(record.get("representative_member_id") or "")
        if representative_id:
            cluster_counts[representative_id] = cluster_counts.get(representative_id, 0) + 1

        refs = record.get("source_references")
        refs = refs if isinstance(refs, dict) else {}
        for key in tracked_keys:
            source = refs.get(key)
            bucket = "geometry_handoff" if source == "geometry_handoff" else "fallback"
            source_counts[key][bucket] += 1

    return {
        "source_counts": source_counts,
        "cluster_member_counts": cluster_counts,
        "all_records_use_geometry_handoff": all(
            counts["fallback"] == 0 for counts in source_counts.values()
        ) if records else False,
    }


def _milling_feature_summary(records: Sequence[Dict[str, object]]) -> Dict[str, object]:
    through_hole_count = 0
    counterbore_count = 0
    slot_count = 0
    slot_widths: List[float] = []
    counterbore_diameters: List[float] = []
    counterbore_auto_diameters: List[float] = []
    counterbore_factors: List[float] = []
    counterbore_sources: List[str] = []
    counterbore_auto_match_flags: List[bool] = []

    for record in records:
        milling_geometry = record.get("milling_geometry")
        milling_geometry = milling_geometry if isinstance(milling_geometry, dict) else {}
        holes = milling_geometry.get("holes")
        holes = holes if isinstance(holes, list) else []
        for hole in holes:
            if not isinstance(hole, dict):
                continue
            if hole.get("type") == "through_hole":
                through_hole_count += 1
            elif hole.get("type") == "flat_bottom_counterbore":
                counterbore_count += 1
        slot = milling_geometry.get("slot")
        if isinstance(slot, dict):
            slot_count += 1
            if slot.get("width") is not None:
                slot_widths.append(float(slot["width"]))
        milling = record.get("milling")
        milling = milling if isinstance(milling, dict) else {}
        if milling.get("counterbore_dia") is not None:
            counterbore_diameters.append(float(milling["counterbore_dia"]))
        if milling.get("counterbore_dia_auto") is not None:
            counterbore_auto_diameters.append(float(milling["counterbore_dia_auto"]))
        if milling.get("counterbore_dia_factor") is not None:
            counterbore_factors.append(float(milling["counterbore_dia_factor"]))
        if milling.get("counterbore_dia_source") is not None:
            counterbore_sources.append(str(milling["counterbore_dia_source"]))
        if milling.get("counterbore_matches_auto") is not None:
            counterbore_auto_match_flags.append(bool(milling["counterbore_matches_auto"]))

    return {
        "through_hole_count": through_hole_count,
        "counterbore_count": counterbore_count,
        "slot_count": slot_count,
        "slot_widths": sorted(set(slot_widths)),
        "counterbore_diameters": sorted(set(counterbore_diameters)),
        "counterbore_auto_diameters": sorted(set(counterbore_auto_diameters)),
        "counterbore_dia_factors": sorted(set(counterbore_factors)),
        "counterbore_dia_sources": sorted(set(counterbore_sources)),
        "counterbores_match_auto_proportion": (
            all(counterbore_auto_match_flags)
            if counterbore_auto_match_flags
            else None
        ),
        "counterbores_per_through_hole": (
            float(counterbore_count) / float(through_hole_count)
            if through_hole_count
            else 0.0
        ),
    }


def export_ct_json(
    geometry_payload: Dict[str, object],
    calc_payload: Dict[str, object],
    out_json_path: Path,
    bottom_face_mode: Optional[str] = None,
    process_only_passed: bool = False,
) -> Dict[str, object]:
    effective_geometry_payload, used_synced_geometry_payload = _effective_geometry_payload(
        geometry_payload,
        calc_payload,
    )

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
    source_geometry_units = _geometry_payload_units(effective_geometry_payload)
    reference_usage = _reference_usage_summary(records)
    milling_feature_summary = _milling_feature_summary(records)

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
            "used_synced_geometry_payload": used_synced_geometry_payload,
            "ct_geometry_mode": "timber_columns_with_milling_cuts",
            "source_geometry_units": source_geometry_units,
            "record_units": "millimeters",
            "inspection_units": source_geometry_units,
            "timber_model_schema_units": "meters",
            "reference_usage": reference_usage,
            "milling_feature_summary": milling_feature_summary,
        },
        "records": records,
        "timber_model_schema": timber_model_schema,
    }

    try:
        out_json_path.parent.mkdir(parents=True, exist_ok=True)
        with out_json_path.open("w", encoding="utf-8") as fp:
            json.dump(package, fp, indent=2)
        package["metadata"]["json_write_succeeded"] = True
        package["metadata"]["json_path"] = str(out_json_path)
    except OSError as exc:
        # GH users still need the in-memory milling package even when a target
        # export file is locked or the folder is not writable.
        package["metadata"]["json_write_succeeded"] = False
        package["metadata"]["json_path"] = str(out_json_path)
        package["metadata"]["json_write_error"] = str(exc)

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
