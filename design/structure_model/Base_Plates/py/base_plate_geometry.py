"""
Base plate geometry scaffold for T2 fabrication.

Module responsibilities:
1) Read the latest line model export from design/line_model/data.
2) Extract column member dimensions, inclinations, grouping, and indices.
3) Build base plate geometry descriptors consumable by calculations and CT export.

This module is written to run in plain CPython and in Grasshopper Python 3.
If Rhino.Geometry is available, optional Brep previews can be produced.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import Rhino.Geometry as rg  # type: ignore
except Exception:
    rg = None


Point3 = Tuple[float, float, float]

SUPPORTED_GEOMETRY_UNITS = {"meters", "millimeters"}
FOOTING_SOURCE_UNITS = "millimeters"
FOOTING_MODEL_UNITS = "meters"
DEFAULT_BOLT_DIA = 0.008
DEFAULT_HOLE_CLEARANCE = 0.001
DEFAULT_CODE_MIN_BOLT_HOLE_DIA = DEFAULT_BOLT_DIA + DEFAULT_HOLE_CLEARANCE
CODE_BASELINE_PLATE_HOLES = {
    "row_count": 3,
    "holes_per_row": 4,
    "diameter": DEFAULT_CODE_MIN_BOLT_HOLE_DIA,
    "row_spacing": 0.060,
    "pitch": 0.100,
    "end_distance": 0.110,
    "row_mode": "double_row",
    "stagger_offset": 0.0,
}
PROJECT_MIN_HEEL_FILLET_RADIUS = 0.020
PROJECT_DEFAULT_HEEL_FILLET_RADIUS = 0.020
SIZING_RECOMMENDATION_KEYS = {
    "bolt_dia",
    "hole_clearance",
    "bolt_hole_dia",
    "code_min_bolt_hole_dia",
    "project_min_bolt_hole_dia",
    "min_bolt_hole_dia",
    "rows",
    "holes_per_row",
    "hole_pattern",
    "stagger_offset",
    "pitch_parallel",
    "gage_perp",
    "end_distance",
    "bottom_end_distance",
    "top_end_distance",
    "bottom_end_distance_multiplier",
    "total_bolt_count",
    "requested_total_bolt_count",
    "actual_total_bolt_count",
    "recommended_total_bolt_count",
    "recommended_holes_per_row",
    "required_total_bolt_count",
    "edge_distance",
    "plate_length",
    "plate_width",
    "plate_thickness",
    "corner_radius",
    "corner_radius_min",
    "corner_radius_code_min",
    "corner_radius_project_min",
    "corner_radius_governing_min",
    "corner_radius_preferred",
}


def _canonical_units(value: object, default: str = "auto") -> str:
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
        "auto": "auto",
    }
    return aliases.get(text, default)


def _canonical_bottom_face_mode(value: object) -> str:
    text = str(value or "Perpendicular_to_grain").strip().lower().replace(" ", "_")
    aliases = {
        "perpendicular_to_grain": "Perpendicular_to_grain",
        "perpendicular": "Perpendicular_to_grain",
        "perp": "Perpendicular_to_grain",
        "parallel_to_ground": "Parallel_to_ground",
        "parallel": "Parallel_to_ground",
        "ground": "Parallel_to_ground",
    }
    return aliases.get(text, "Perpendicular_to_grain")


def _canonical_plate_hole_pattern(value: object) -> str:
    try:
        numeric = float(value)
        text = str(int(numeric)) if numeric.is_integer() else str(value or "").strip().lower()
    except Exception:
        text = str(value or "").strip().lower()
    text = text.replace(" ", "_")
    aliases = {
        "1": "single_row_centerline",
        "single": "single_row_centerline",
        "single_row": "single_row_centerline",
        "single_row_centerline": "single_row_centerline",
        "centerline": "single_row_centerline",
        "2": "double_row",
        "double": "double_row",
        "double_row": "double_row",
        "rectangular": "double_row",
        "rectangular_double_row": "double_row",
        "3": "staggered_double_row",
        "stagger": "staggered_double_row",
        "staggered": "staggered_double_row",
        "staggered_double_row": "staggered_double_row",
    }
    return aliases.get(text, "")


def _median_scalar_values(values: Sequence[float], default: float = 0.0) -> float:
    items = sorted(float(value) for value in values)
    if not items:
        return default
    mid = len(items) // 2
    if len(items) % 2:
        return items[mid]
    return 0.5 * (items[mid - 1] + items[mid])


def _infer_member_units(members: Sequence["MemberRecord"]) -> str:
    widths = [abs(float(member.width)) for member in members if member.width is not None]
    median_width = _median_scalar_values(widths, 0.0)
    # The current line-model exports use either ~0.08-0.12 m or ~80-120 mm
    # timber widths, which gives a very reliable split without relying on filenames.
    return "meters" if 0.0 < median_width < 10.0 else "millimeters"


def _resolve_geometry_units(members: Sequence["MemberRecord"], requested_units: object = "auto") -> str:
    units = _canonical_units(requested_units, "auto")
    if units in SUPPORTED_GEOMETRY_UNITS:
        return units
    return _infer_member_units(members)


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


def _scale_point(point: Point3, factor: float) -> Point3:
    return (point[0] * factor, point[1] * factor, point[2] * factor)


def _scale_geometry_copy(geometry: object, factor: float) -> object:
    if rg is None or geometry is None or abs(factor - 1.0) <= 1e-12:
        return geometry

    duplicate = getattr(geometry, "Duplicate", None)
    scaled = duplicate() if callable(duplicate) else None
    if scaled is None:
        duplicate_brep = getattr(geometry, "DuplicateBrep", None)
        scaled = duplicate_brep() if callable(duplicate_brep) else None
    if scaled is None:
        return geometry

    try:
        scaled.Transform(rg.Transform.Scale(rg.Plane.WorldXY, factor))
        return scaled
    except Exception:
        return geometry


def _scale_xyz_tuple(value: Sequence[float], factor: float) -> Tuple[float, ...]:
    return tuple(float(item) * factor for item in value)


def _extract_sizing_recommendations(value: object) -> Dict[str, object]:
    if not isinstance(value, dict):
        return {}
    if any(key in value for key in SIZING_RECOMMENDATION_KEYS):
        return dict(value)

    direct = value.get("sizing_recommendations")
    if isinstance(direct, dict):
        return dict(direct)

    engineering = value.get("engineering")
    if isinstance(engineering, dict):
        nested = engineering.get("sizing_recommendations")
        if isinstance(nested, dict):
            return dict(nested)
    return {}


def _scale_sizing_recommendations(
    value: object,
    from_units: str = "millimeters",
    to_units: str = FOOTING_MODEL_UNITS,
) -> Dict[str, object]:
    sizing = _extract_sizing_recommendations(value)
    if not sizing:
        return {}

    factor = _unit_scale_factor(from_units, to_units)
    scaled = dict(sizing)
    for key in (
        "bolt_dia",
        "hole_clearance",
        "bolt_hole_dia",
        "code_min_bolt_hole_dia",
        "project_min_bolt_hole_dia",
        "min_bolt_hole_dia",
        "pitch_parallel",
        "gage_perp",
        "end_distance",
        "bottom_end_distance",
        "top_end_distance",
        "stagger_offset",
        "edge_distance",
        "plate_length",
        "plate_width",
        "plate_thickness",
        "corner_radius",
        "corner_radius_min",
        "corner_radius_code_min",
        "corner_radius_project_min",
        "corner_radius_governing_min",
        "corner_radius_preferred",
    ):
        if scaled.get(key) is not None:
            scaled[key] = float(scaled[key]) * factor
    return scaled


def _scale_footing_reference_data_to_meters() -> None:
    factor = _unit_scale_factor(FOOTING_SOURCE_UNITS, FOOTING_MODEL_UNITS)
    global SOURCE_BASE_CENTER
    global SOURCE_WEB_PLATE_CENTER
    global SOURCE_FULL_PLATE_CENTER
    global SOURCE_STIFFENER_CENTER
    if abs(factor - 1.0) <= 1e-12:
        return

    SOURCE_BASE_CENTER = _scale_xyz_tuple(SOURCE_BASE_CENTER, factor)  # type: ignore[assignment]
    SOURCE_WEB_PLATE_CENTER = _scale_xyz_tuple(SOURCE_WEB_PLATE_CENTER, factor)  # type: ignore[assignment]
    SOURCE_FULL_PLATE_CENTER = _scale_xyz_tuple(SOURCE_FULL_PLATE_CENTER, factor)  # type: ignore[assignment]
    SOURCE_STIFFENER_CENTER = _scale_xyz_tuple(SOURCE_STIFFENER_CENTER, factor)  # type: ignore[assignment]

    for key in ("base_diameter", "base_length", "base_width", "base_thickness", "hole_diameter"):
        REFERENCE[key] = float(REFERENCE[key]) * factor
    REFERENCE["hole_spacing"] = _scale_xyz_tuple(REFERENCE["hole_spacing"], factor)
    REFERENCE["edge_spacing"] = _scale_xyz_tuple(REFERENCE["edge_spacing"], factor)
    for spec in REFERENCE["plates"]:
        spec["center"] = _scale_xyz_tuple(spec["center"], factor)
        for key in ("length", "width", "thickness"):
            spec[key] = float(spec[key]) * factor
        if spec.get("support_heel_local") is not None:
            spec["support_heel_local"] = _scale_xyz_tuple(spec["support_heel_local"], factor)
        spec["edge_lengths"] = _scale_xyz_tuple(spec["edge_lengths"], factor)

    for spec in FULL_PLATE_REFERENCE:
        spec["center"] = _scale_xyz_tuple(spec["center"], factor)
        for key in ("full_length", "straight_length", "width", "thickness"):
            spec[key] = float(spec[key]) * factor

    for spec in STIFFENER_REFERENCE:
        spec["center"] = _scale_xyz_tuple(spec["center"], factor)
        for key in ("length", "height", "low_height", "thickness"):
            spec[key] = float(spec[key]) * factor
        spec["profile_points"] = tuple(
            _scale_xyz_tuple(point, factor)
            for point in spec["profile_points"]
        )

    for spec in PLATE_HOLE_REFERENCE:
        spec["center"] = _scale_xyz_tuple(spec["center"], factor)
        for key in ("diameter", "row_spacing", "pitch"):
            spec[key] = float(spec[key]) * factor


@dataclass
class MemberRecord:
    member_id: str
    index: int
    group: Optional[int]
    level: Optional[int]
    hierarchy: Optional[str]
    etype: Optional[str]
    width: float
    height: float
    start: Point3
    end: Point3
    length: float
    direction: Point3
    azimuth_deg: float
    inclination_deg: float
    support_node_id: Optional[str] = None
    support_cut_plane_z: Optional[float] = None
    support_cut_plane_point: Optional[Point3] = None


@dataclass
class BasePlateRecord:
    member_id: str
    member_index: int
    group: Optional[int]
    center: Point3
    normal: Point3
    x_axis: Point3
    y_axis: Point3
    length: float
    width: float
    thickness: float
    corners: List[Point3]


def _scale_member_records(
    members: Sequence[MemberRecord],
    from_units: str,
    to_units: str,
) -> List[MemberRecord]:
    factor = _unit_scale_factor(from_units, to_units)
    if abs(factor - 1.0) <= 1e-12:
        return list(members)
    return [
        MemberRecord(
            member_id=member.member_id,
            index=member.index,
            group=member.group,
            level=member.level,
            hierarchy=member.hierarchy,
            etype=member.etype,
            width=member.width * factor,
            height=member.height * factor,
            start=_scale_point(member.start, factor),
            end=_scale_point(member.end, factor),
            length=member.length * factor,
            direction=member.direction,
            azimuth_deg=member.azimuth_deg,
            inclination_deg=member.inclination_deg,
            support_node_id=member.support_node_id,
            support_cut_plane_z=None
            if member.support_cut_plane_z is None
            else member.support_cut_plane_z * factor,
            support_cut_plane_point=None
            if member.support_cut_plane_point is None
            else _scale_point(member.support_cut_plane_point, factor),
        )
        for member in members
    ]


def _vector(a: Point3, b: Point3) -> Point3:
    return (b[0] - a[0], b[1] - a[1], b[2] - a[2])


def _dot(a: Point3, b: Point3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Point3, b: Point3) -> Point3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _length(v: Point3) -> float:
    return math.sqrt(_dot(v, v))


def _unit(v: Point3, fallback: Point3 = (1.0, 0.0, 0.0)) -> Point3:
    lv = _length(v)
    if lv <= 1e-9:
        return fallback
    return (v[0] / lv, v[1] / lv, v[2] / lv)


def _add(a: Point3, b: Point3) -> Point3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(v: Point3, s: float) -> Point3:
    return (v[0] * s, v[1] * s, v[2] * s)


def _distance(a: Point3, b: Point3) -> float:
    return _length(_vector(a, b))


def _debug_xyz(value):
    if value is None:
        return None
    if hasattr(value, "X") and hasattr(value, "Y") and hasattr(value, "Z"):
        return (float(value.X), float(value.Y), float(value.Z))
    return value


def _line_angles(direction: Point3) -> Tuple[float, float]:
    d = _unit(direction)
    azimuth = math.degrees(math.atan2(d[1], d[0]))
    inclination = math.degrees(math.atan2(d[2], math.sqrt(d[0] ** 2 + d[1] ** 2)))
    return azimuth, inclination


def _is_compas_line_obj(value: object) -> bool:
    if not isinstance(value, dict):
        return False
    return value.get("dtype") == "compas.geometry/Line" and isinstance(value.get("data"), dict)


def _line_points_from_compas_obj(value: Dict[str, object]) -> Optional[Tuple[Point3, Point3]]:
    data = value.get("data")
    if not isinstance(data, dict):
        return None
    start = data.get("start")
    end = data.get("end")
    if not (isinstance(start, Sequence) and isinstance(end, Sequence) and len(start) >= 3 and len(end) >= 3):
        return None
    try:
        return (
            (float(start[0]), float(start[1]), float(start[2])),
            (float(end[0]), float(end[1]), float(end[2])),
        )
    except Exception:
        return None


def _candidate_line_export_paths(root: Path) -> List[Path]:
    data_dir = root / "design" / "line_model" / "data"
    preferred = [
        data_dir / "260518_v3_line_model.json",
        data_dir / "meters_shifted_lines.json",
        data_dir / "0806_shifted_lines.json",
        data_dir / "shifted_lines.json",
    ]
    candidates: List[Path] = [p for p in preferred if p.exists()]
    if data_dir.exists():
        for p in sorted(data_dir.glob("*line_model*.json")):
            if p not in candidates:
                candidates.append(p)
        for p in sorted(data_dir.glob("*shifted_lines*.json")):
            if p not in candidates:
                candidates.append(p)
    return candidates


def resolve_latest_line_model_path(project_root: Optional[Path] = None) -> Path:
    if project_root is not None:
        root = project_root
    else:
        module_path = Path(__file__).resolve()
        root = next(
            (
                parent
                for parent in module_path.parents
                if (parent / "design" / "line_model" / "data").exists()
            ),
            module_path.parents[4],
        )
    candidates = _candidate_line_export_paths(root)
    if not candidates:
        raise FileNotFoundError("No shifted line-model export found under design/line_model/data")
    return max(candidates, key=lambda p: p.stat().st_mtime)


def _extract_members_from_compas_graph(raw: Dict[str, object]) -> List[MemberRecord]:
    data = raw.get("data")
    if not isinstance(data, dict):
        return []

    edge_map = data.get("edge")
    if not isinstance(edge_map, dict):
        return []
    node_map = data.get("node")
    if not isinstance(node_map, dict):
        node_map = {}

    def _node_cut_plane_point(node_key) -> Optional[Point3]:
        attrs = node_map.get(str(node_key))
        if not isinstance(attrs, dict):
            return None
        cut_plane = attrs.get("cut_plane")
        if not isinstance(cut_plane, dict):
            return None
        plane_data = cut_plane.get("data")
        if not isinstance(plane_data, dict):
            return None
        point = plane_data.get("point")
        if not (isinstance(point, Sequence) and len(point) >= 3):
            return None
        try:
            return (float(point[0]), float(point[1]), float(point[2]))
        except Exception:
            return None

    records: List[MemberRecord] = []
    idx = 0
    for u_key, nbrs in edge_map.items():
        if not isinstance(nbrs, dict):
            continue
        for v_key, attrs in nbrs.items():
            if not isinstance(attrs, dict):
                continue

            shifted = attrs.get("shifted_line")
            base = attrs.get("line")
            line_obj = shifted if _is_compas_line_obj(shifted) else base
            if not _is_compas_line_obj(line_obj):
                continue

            pts = _line_points_from_compas_obj(line_obj)  # type: ignore[arg-type]
            if pts is None:
                continue
            start, end = pts
            direction = _vector(start, end)
            length = _length(direction)
            if length <= 1e-9:
                continue

            azimuth, inclination = _line_angles(direction)
            width = float(attrs.get("width") or 100.0)
            height = float(attrs.get("height") or 140.0)
            group_raw = attrs.get("group")
            group = int(group_raw) if group_raw is not None else None
            level_raw = attrs.get("level")
            level = int(level_raw) if level_raw is not None else None

            member_id = "{0}-{1}".format(u_key, v_key)
            u_cut_plane_point = _node_cut_plane_point(u_key)
            v_cut_plane_point = _node_cut_plane_point(v_key)
            support_node_id = None
            support_cut_plane_z = None
            support_cut_plane_point = None
            if u_cut_plane_point is not None:
                support_node_id = str(u_key)
                support_cut_plane_point = u_cut_plane_point
                support_cut_plane_z = u_cut_plane_point[2]
            elif v_cut_plane_point is not None:
                support_node_id = str(v_key)
                support_cut_plane_point = v_cut_plane_point
                support_cut_plane_z = v_cut_plane_point[2]
            records.append(
                MemberRecord(
                    member_id=member_id,
                    index=idx,
                    group=group,
                    level=level,
                    hierarchy=str(attrs.get("hierarchy")) if attrs.get("hierarchy") is not None else None,
                    etype=str(attrs.get("etype")) if attrs.get("etype") is not None else None,
                    width=width,
                    height=height,
                    start=start,
                    end=end,
                    length=length,
                    direction=_unit(direction),
                    azimuth_deg=azimuth,
                    inclination_deg=inclination,
                    support_node_id=support_node_id,
                    support_cut_plane_z=support_cut_plane_z,
                    support_cut_plane_point=support_cut_plane_point,
                )
            )
            idx += 1
    return records


def load_member_records(line_model_path: Optional[Path] = None) -> List[MemberRecord]:
    path = line_model_path or resolve_latest_line_model_path()
    with path.open("r", encoding="utf-8") as fp:
        raw = json.load(fp)

    # Primary support: graph-style export with edge attributes.
    members = _extract_members_from_compas_graph(raw)
    return members


def _base_end_point(member: MemberRecord) -> Point3:
    # "Base" end is assumed as the lower-Z endpoint.
    return member.start if member.start[2] <= member.end[2] else member.end


def _plane_axes_from_member(
    member: MemberRecord,
    bottom_face_mode: str,
) -> Tuple[Point3, Point3, Point3]:
    mode = _canonical_bottom_face_mode(bottom_face_mode)
    axis = _unit(member.direction)

    if mode == "Parallel_to_ground":
        normal = (0.0, 0.0, 1.0)
        x_axis_xy = (axis[0], axis[1], 0.0)
        if _length(x_axis_xy) <= 1e-9:
            x_axis = (1.0, 0.0, 0.0)
        else:
            x_axis = _unit(x_axis_xy)
        y_axis = _unit(_cross(normal, x_axis), fallback=(0.0, 1.0, 0.0))
        return normal, x_axis, y_axis

    normal = axis
    ref = (0.0, 0.0, 1.0)
    y_axis = _cross(normal, ref)
    if _length(y_axis) <= 1e-9:
        ref = (1.0, 0.0, 0.0)
        y_axis = _cross(normal, ref)
    y_axis = _unit(y_axis, fallback=(0.0, 1.0, 0.0))
    x_axis = _unit(_cross(y_axis, normal), fallback=(1.0, 0.0, 0.0))
    return normal, x_axis, y_axis


def _rectangle_corners(center: Point3, x_axis: Point3, y_axis: Point3, lx: float, ly: float) -> List[Point3]:
    hx = 0.5 * lx
    hy = 0.5 * ly
    offsets = [
        _add(_scale(x_axis, hx), _scale(y_axis, hy)),
        _add(_scale(x_axis, -hx), _scale(y_axis, hy)),
        _add(_scale(x_axis, -hx), _scale(y_axis, -hy)),
        _add(_scale(x_axis, hx), _scale(y_axis, -hy)),
    ]
    return [_add(center, offset) for offset in offsets]


def build_base_plate_records(
    members: Iterable[MemberRecord],
    plate_length: float = 0.8,
    plate_width: float = 0.8,
    plate_thickness: float = 0.02,
    bottom_face_mode: str = "Perpendicular_to_grain",
    center_overrides: Optional[Dict[str, Point3]] = None,
) -> List[BasePlateRecord]:
    records: List[BasePlateRecord] = []
    center_overrides = center_overrides or {}
    for member in members:
        center = center_overrides.get(member.member_id, _base_end_point(member))
        normal, x_axis, y_axis = _plane_axes_from_member(member, bottom_face_mode)
        corners = _rectangle_corners(center, x_axis, y_axis, plate_length, plate_width)
        records.append(
            BasePlateRecord(
                member_id=member.member_id,
                member_index=member.index,
                group=member.group,
                center=center,
                normal=normal,
                x_axis=x_axis,
                y_axis=y_axis,
                length=plate_length,
                width=plate_width,
                thickness=plate_thickness,
                corners=corners,
            )
        )
    return records


def to_payload_dict(
    members: Sequence[MemberRecord],
    base_plates: Sequence[BasePlateRecord],
    line_model_path: Path,
    bottom_face_mode: str,
    source_member_count: Optional[int] = None,
    target_support_nodes_only: bool = False,
    deduplicate_support_nodes: bool = False,
    target_member_index: Optional[int] = None,
    target_member_index_mode: Optional[str] = None,
    target_cluster_index: Optional[int] = None,
    geometry_kind: str = "flat_plate",
    geometry_units: str = FOOTING_MODEL_UNITS,
    source_geometry_units: Optional[str] = None,
    footing_source_units: Optional[str] = None,
    footing_output_units: Optional[str] = None,
    footing_sizing_source: str = "code_baseline",
    footing_sizing_recommendations: Optional[Dict[str, object]] = None,
    support_cluster_member_indices: Optional[Dict[str, List[int]]] = None,
    footing_breps: Optional[Sequence[object]] = None,
    footing_debug: Optional[Sequence[Dict[str, object]]] = None,
    footing_handoffs: Optional[Sequence[Dict[str, object]]] = None,
) -> Dict[str, object]:
    grouped: Dict[str, int] = {}
    for m in members:
        key = str(m.group) if m.group is not None else "None"
        grouped[key] = grouped.get(key, 0) + 1

    footing_handoff_records = list(footing_handoffs or [])
    active_footing_record = footing_handoff_records[0] if footing_handoff_records else {}
    active_handoff = (
        dict(active_footing_record.get("metadata") or {})
        if isinstance(active_footing_record, dict)
        else {}
    )

    annotation_payload = dict(active_handoff.get("annotation") or {})
    layout_payload = dict(active_handoff.get("layout") or {})
    payload = {
        "metadata": {
            "module": "base_plate_geometry",
            "schema_version": "base_plate_geometry_payload/v2",
            "line_model_path": str(line_model_path),
            "generated_utc": datetime.utcnow().isoformat() + "Z",
            "member_count": len(members),
            "source_member_count": source_member_count if source_member_count is not None else len(members),
            "group_counts": grouped,
            "bottom_face_mode": bottom_face_mode,
            "target_support_nodes_only": target_support_nodes_only,
            "deduplicate_support_nodes": deduplicate_support_nodes,
            "target_member_index": target_member_index,
            "target_member_index_mode": target_member_index_mode,
            "target_cluster_index": target_cluster_index,
            "geometry_kind": geometry_kind,
            "geometry_units": geometry_units,
            "source_geometry_units": source_geometry_units,
            "footing_source_units": footing_source_units,
            "footing_output_units": footing_output_units,
            "footing_sizing_source": footing_sizing_source,
            "footing_sizing_recommendations": footing_sizing_recommendations or {},
            "support_cluster_member_indices": support_cluster_member_indices or {},
            "footing_debug": list(footing_debug or []),
            "footing_handoff_count": len(footing_handoff_records),
            "active_footing_member_id": active_footing_record.get("member_id")
            if isinstance(active_footing_record, dict)
            else None,
        },
        "members": [asdict(m) for m in members],
        "base_plates": [asdict(p) for p in base_plates],
        "footing_breps": list(footing_breps or []),
        "footings": footing_handoff_records,
        "handoff": active_handoff,
        "annotation_metadata": active_handoff,
        "annotation_payload": annotation_payload,
        "layout_payload": layout_payload,
    }
    # Mirror the preferred handoff sections at top level so downstream GH
    # components can consume a stable shape without searching nested records.
    for section in ("geometry", "milling", "forces", "checks", "installation", "references"):
        payload[section] = dict(active_handoff.get(section) or {})
    return payload


def _member_center(member: MemberRecord, center_overrides: Optional[Dict[str, Point3]] = None) -> Point3:
    if center_overrides and member.member_id in center_overrides:
        return center_overrides[member.member_id]
    return _base_end_point(member)


def _support_direction_angles(member: MemberRecord) -> Tuple[float, float]:
    base = _base_end_point(member)
    other = member.end if base == member.start else member.start
    direction = _vector(base, other)
    return _line_angles(direction)


def _angle_delta_deg(a: float, b: float) -> float:
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def _matched_specs_by_azimuth(
    source_specs: Sequence[Dict[str, object]],
    target_specs: Sequence[Dict[str, object]],
    target_key: str = "source_azimuth_deg",
) -> List[Tuple[int, Dict[str, object]]]:
    remaining = list(enumerate(source_specs or []))
    matched: List[Tuple[int, Dict[str, object]]] = []
    for target in target_specs or []:
        if not remaining:
            break
        target_azimuth = target.get(target_key, target.get("azimuth_deg"))
        if target_azimuth is None:
            matched.append(remaining.pop(0))
            continue
        best = min(
            remaining,
            key=lambda item: _angle_delta_deg(
                item[1].get("azimuth_deg", 0.0),
                target_azimuth,
            ),
        )
        matched.append(best)
        remaining.remove(best)
    return matched


def _incident_members_for_plate_slots(
    incident_members: Sequence[MemberRecord],
) -> List[MemberRecord]:
    remaining = list(incident_members)
    ordered: List[MemberRecord] = []
    if not remaining:
        return ordered

    reference_azimuths = [float(spec["azimuth_deg"]) for spec in REFERENCE["plates"]]
    for reference_azimuth in reference_azimuths:
        if not remaining:
            break
        best = min(
            remaining,
            key=lambda member: _angle_delta_deg(
                _support_direction_angles(member)[0],
                reference_azimuth,
            ),
        )
        ordered.append(best)
        remaining.remove(best)

    ordered.extend(sorted(remaining, key=lambda member: _support_direction_angles(member)[0]))
    return ordered


def _apply_target_cluster_filter(
    members: Sequence[MemberRecord],
    center_overrides: Dict[str, Point3],
    target_cluster_index: Optional[int],
) -> Tuple[List[MemberRecord], Dict[str, Point3], Optional[str]]:
    if target_cluster_index is None:
        return list(members), center_overrides, None

    idx = int(target_cluster_index)
    ordered = sorted(members, key=lambda m: m.index)

    if 1 <= idx <= len(ordered):
        member = ordered[idx - 1]
        next_members = [member]
        next_overrides = {
            member_id: center
            for member_id, center in center_overrides.items()
            if member_id == member.member_id
        }
        return next_members, next_overrides, "cluster_index"

    return [], {}, "cluster_not_found"


def _build_footing_breps_for_members(
    members: Sequence[MemberRecord],
    center_overrides: Dict[str, Point3],
    cluster_members_by_representative: Optional[Dict[str, Sequence[MemberRecord]]] = None,
    sizing_recommendations: Optional[Dict[str, object]] = None,
    bottom_face_mode: str = "Perpendicular_to_grain",
    include_stiffeners: Optional[bool] = None,
    baseplate_top_z: Optional[float] = None,
    base_plate_brep: Optional[object] = None,
    base_diameter: Optional[float] = None,
    min_hole_edge_spacing: Optional[float] = None,
    heel_fillet_radius: Optional[float] = None,
    timber_bottom_gap: Optional[float] = None,
    min_timber_gap: Optional[float] = None,
    bottom_end_distance_multiplier: Optional[float] = None,
    webplate_thickness: Optional[float] = None,
    webplate_hole_diameter: Optional[float] = None,
    webplate_hole_pitch: Optional[float] = None,
    webplate_hole_transverse_spacing: Optional[float] = None,
    webplate_hole_rows: Optional[int] = None,
    webplate_hole_pattern: Optional[object] = None,
    webplate_hole_stagger_offset: Optional[float] = None,
    bolt_dia: Optional[float] = None,
    hole_clearance: Optional[float] = None,
    total_bolt_count: Optional[int] = None,
    stiffener_pair_axis_shift: Optional[float] = None,
    stiffener_pair_from_point: Optional[object] = None,
    stiffener_pair_to_point: Optional[object] = None,
    geometry_units: str = FOOTING_MODEL_UNITS,
) -> Tuple[List[object], List[Dict[str, object]], List[Dict[str, object]]]:
    if rg is None or not members:
        return [], [], []
    footing_run = globals().get("base_footing_run")
    if not callable(footing_run):
        return [], [], []

    member_to_footing = _unit_scale_factor(geometry_units, SOURCE_UNITS)
    footing_to_member = _unit_scale_factor(SOURCE_UNITS, geometry_units)
    breps: List[object] = []
    debug_records: List[Dict[str, object]] = []
    handoff_records: List[Dict[str, object]] = []
    for member in members:
        cx, cy, cz = _member_center(member, center_overrides)
        center_in_footing_units = _scale_point((cx, cy, cz), member_to_footing)
        plane = rg.Plane(rg.Point3d(*center_in_footing_units), rg.Vector3d.XAxis, rg.Vector3d.YAxis)
        incident_members = list(
            (cluster_members_by_representative or {}).get(member.member_id) or [member]
        )
        ordered_incident_members = _incident_members_for_plate_slots(incident_members)
        plate_azimuths = []
        plate_altitudes = []
        plate_collision_zs = []
        plate_collision_points = []
        plate_timber_widths = []
        plate_timber_heights = []
        for incident_member in ordered_incident_members:
            azimuth, altitude = _support_direction_angles(incident_member)
            plate_azimuths.append(azimuth)
            plate_altitudes.append(altitude)
            plate_collision_zs.append(incident_member.support_cut_plane_z)
            plate_collision_points.append(incident_member.support_cut_plane_point)
            plate_timber_widths.append(incident_member.width)
            plate_timber_heights.append(incident_member.height)
        try:
            result = footing_run(
                base_plane=plane,
                ref_base_plate_brep=base_plate_brep,
                plate_azimuths=plate_azimuths,
                plate_altitudes=plate_altitudes,
                plate_collision_zs=plate_collision_zs,
                plate_collision_points=plate_collision_points,
                plate_timber_widths=plate_timber_widths,
                plate_timber_heights=plate_timber_heights,
                plate_member_indices=[incident_member.index for incident_member in ordered_incident_members],
                plate_member_ids=[incident_member.member_id for incident_member in ordered_incident_members],
                plate_orientation_sources=["line_model_member"] * len(plate_azimuths),
                sizing_recommendations=sizing_recommendations,
                bottom_face_mode=bottom_face_mode,
                include_stiffeners=include_stiffeners,
                baseplate_top_z=baseplate_top_z,
                base_diameter=base_diameter,
                min_hole_edge_spacing=min_hole_edge_spacing,
                heel_fillet_radius=heel_fillet_radius,
                timber_bottom_gap=timber_bottom_gap,
                min_timber_gap=min_timber_gap,
                bottom_end_distance_multiplier=bottom_end_distance_multiplier,
                plate_thicknesses=webplate_thickness,
                plate_hole_diameters=webplate_hole_diameter,
                plate_hole_pitches=webplate_hole_pitch,
                plate_hole_row_spacings=webplate_hole_transverse_spacing,
                plate_hole_rows=webplate_hole_rows,
                plate_hole_patterns=webplate_hole_pattern,
                plate_hole_stagger_offsets=webplate_hole_stagger_offset,
                plate_bolt_diameters=bolt_dia,
                plate_hole_clearances=hole_clearance,
                plate_total_hole_counts=total_bolt_count,
                stiffener_pair_axis_shift=stiffener_pair_axis_shift,
                stiffener_pair_from_point=stiffener_pair_from_point,
                stiffener_pair_to_point=stiffener_pair_to_point,
                enabled=True,
            )
        except Exception as exc:
            debug_records.append(
                {
                    "member_id": member.member_id,
                    "member_index": member.index,
                    "incident_member_ids": [
                        incident_member.member_id
                        for incident_member in ordered_incident_members
                    ],
                    "incident_member_indices": [
                        incident_member.index
                        for incident_member in ordered_incident_members
                    ],
                    "error": "base_footing_run failed: {0}".format(exc),
                    "exception_type": type(exc).__name__,
                    "plate_azimuths": list(plate_azimuths),
                    "plate_altitudes": list(plate_altitudes),
                    "webplate_hole_rows": webplate_hole_rows,
                    "webplate_hole_pattern": webplate_hole_pattern,
                    "bolt_dia": bolt_dia,
                    "total_bolt_count": total_bolt_count,
                }
            )
            continue

        if not isinstance(result, dict):
            continue
        params = result.get("params") or {}
        verification = result.get("verification") or {}
        handoff_metadata = result.get("metadata") or {}
        handoff_records.append(
            {
                "member_id": member.member_id,
                "member_index": member.index,
                "incident_member_ids": [incident_member.member_id for incident_member in ordered_incident_members],
                "incident_member_indices": [incident_member.index for incident_member in ordered_incident_members],
                "incident_members": [asdict(incident_member) for incident_member in ordered_incident_members],
                "metadata": handoff_metadata,
            }
        )
        debug_records.append(
            {
                "member_id": member.member_id,
                "plate_brep_count": len(result.get("plate_breps") or []),
                "full_plate_brep_count": len(result.get("full_plate_breps") or []),
                "merged_footing_count": len(result.get("merged_footing_breps") or []),
                "active_web_plate_source": params.get("active_web_plate_source"),
                "active_web_plate_count": verification.get("active_web_plate_count"),
                "base_web_difference_succeeded": verification.get("base_web_difference_succeeded"),
                "web_plate_trim_succeeded": verification.get("web_plate_trim_succeeded"),
                "messages": list(verification.get("messages") or []),
                "plate_body_hole_diagnostics": verification.get("plate_body_hole_diagnostics"),
                "plate_hole_pattern_diagnostics": params.get("plate_hole_pattern_diagnostics"),
                "stiffener_specs": [
                    {
                        "stiffener_kind": spec.get("stiffener_kind"),
                        "target_source": spec.get("target_source"),
                        "target_anchor_member_id": spec.get("target_anchor_member_id"),
                        "target_anchor_member_index": spec.get("target_anchor_member_index"),
                        "azimuth_deg": spec.get("azimuth_deg"),
                        "center": _debug_xyz(spec.get("center")),
                        "timber_width": spec.get("timber_width"),
                        "timber_height": spec.get("timber_height"),
                        "webplate_depth": spec.get("webplate_depth"),
                        "webplate_intersection_source": spec.get("webplate_intersection_source"),
                        "collision_point_input": _debug_xyz(spec.get("collision_point_input")),
                        "collision_centerline_point": _debug_xyz(spec.get("collision_centerline_point")),
                        "projected_collision_point": _debug_xyz(spec.get("projected_collision_point")),
                        "edge_projected_collision_point": _debug_xyz(spec.get("edge_projected_collision_point")),
                        "edge_projected_collision_name": spec.get("edge_projected_collision_name"),
                        "heel_face_midline_start": _debug_xyz(spec.get("heel_face_midline_start")),
                        "heel_face_midline_end": _debug_xyz(spec.get("heel_face_midline_end")),
                        "heel_face_midpoint_at_timber_face": _debug_xyz(spec.get("heel_face_midpoint_at_timber_face")),
                        "heel_face_intersection_param": spec.get("heel_face_intersection_param"),
                        "webplate_intersection_target": _debug_xyz(spec.get("webplate_intersection_target")),
                        "unsnapped_pair_interface_tip": _debug_xyz(spec.get("unsnapped_pair_interface_tip")),
                        "unsnapped_pair_shared_edge_midpoint": _debug_xyz(spec.get("unsnapped_pair_shared_edge_midpoint")),
                        "snapped_pair_shared_edge_midpoint": _debug_xyz(spec.get("snapped_pair_shared_edge_midpoint")),
                        "stiffener_snap_residual": spec.get("stiffener_snap_residual"),
                        "stiffener_snap_residual_ok": spec.get("stiffener_snap_residual_ok"),
                        "bottom_face_unflipped_center": _debug_xyz(spec.get("bottom_face_unflipped_center")),
                        "bottom_face_flip_about_intersection_edge": spec.get("bottom_face_flip_about_intersection_edge"),
                        "heel_edge_centerline_point": _debug_xyz(spec.get("heel_edge_centerline_point")),
                        "timber_bottom_face_heel_intersection_point": _debug_xyz(spec.get("timber_bottom_face_heel_intersection_point")),
                        "pair_snap_vector": _debug_xyz(spec.get("pair_snap_vector")),
                        "timber_face_anchor": _debug_xyz(spec.get("timber_face_anchor")),
                        "timber_face_anchor_source": spec.get("timber_face_anchor_source"),
                        "bottom_face_pivot_point": _debug_xyz(spec.get("bottom_face_pivot_point")),
                        "bottom_face_pivot_edge_name": spec.get("bottom_face_pivot_edge_name"),
                        "bottom_face_pivot_edge_start": _debug_xyz(spec.get("bottom_face_pivot_edge_start")),
                        "bottom_face_pivot_edge_end": _debug_xyz(spec.get("bottom_face_pivot_edge_end")),
                        "bottom_face_pivot_edge_parameter": spec.get("bottom_face_pivot_edge_parameter"),
                        "bottom_face_pivot_source": spec.get("bottom_face_pivot_source"),
                        "timber_face_dividing_plane_origin": _debug_xyz(spec.get("timber_face_dividing_plane_origin")),
                        "side_mount_sign": spec.get("side_mount_sign"),
                        "side_mount_span": spec.get("side_mount_span"),
                        "side_mount_center_offset": spec.get("side_mount_center_offset"),
                        "heel_side_alignment_y": spec.get("heel_side_alignment_y"),
                        "toe_side_alignment_y": spec.get("toe_side_alignment_y"),
                        "bottom_face_center_shift": spec.get("bottom_face_center_shift"),
                        "pair_axis_shift": spec.get("pair_axis_shift"),
                        "pair_axis_shift_source": spec.get("pair_axis_shift_source"),
                        "pair_axis_shift_vector": _debug_xyz(spec.get("pair_axis_shift_vector")),
                        "face_intersection_tip_y": spec.get("face_intersection_tip_y"),
                    }
                    for spec in (params.get("stiffener_specs") or [])
                ],
                "plate_orientations": [
                    {
                        "member_index": spec.get("member_index"),
                        "member_id": spec.get("member_id"),
                        "azimuth_deg": spec.get("azimuth_deg"),
                        "inclination_deg": spec.get("inclination_deg"),
                        "orientation_source": spec.get("orientation_source"),
                        "center_alignment_source": spec.get("center_alignment_source"),
                        "heel_datum_source": spec.get("heel_datum_source"),
                        "heel_axis_shift": spec.get("heel_axis_shift"),
                        "collision_z": spec.get("collision_z"),
                        "raw_collision_gap": spec.get("raw_collision_gap"),
                        "effective_collision_gap": spec.get("effective_collision_gap"),
                        "effective_collision_z": spec.get("effective_collision_z"),
                        "effective_timber_bottom_z": spec.get("effective_timber_bottom_z"),
                        "collision_height_from_baseplate": spec.get("collision_height_from_baseplate"),
                        "collision_axis_distance": spec.get("collision_axis_distance"),
                        "code_pattern_length": spec.get("code_pattern_length"),
                        "effective_plate_length": spec.get("effective_plate_length"),
                        "required_heel_to_tip_distance": spec.get("required_heel_to_tip_distance"),
                        "first_hole_axis_offset": spec.get("first_hole_axis_offset"),
                        "last_hole_axis_offset": spec.get("last_hole_axis_offset"),
                        "bottom_hole_end_distance": spec.get("bottom_hole_end_distance"),
                        "top_hole_end_distance": spec.get("top_hole_end_distance"),
                        "bottom_end_distance_multiplier": spec.get("bottom_end_distance_multiplier"),
                        "hole_pattern_span": spec.get("hole_pattern_span"),
                        "hole_stagger_offset": spec.get("hole_stagger_offset"),
                        "top_end_distance_from_last_hole": spec.get("top_end_distance_from_last_hole"),
                        "heel_fillet_radius": spec.get("heel_fillet_radius"),
                        "plate_build_mode": spec.get("plate_build_mode"),
                    }
                    for spec in (params.get("plate_specs") or [])
                ],
            }
        )
        merged_footing_breps = result.get("merged_footing_breps") or []
        if merged_footing_breps:
            for brep in merged_footing_breps:
                if brep is not None:
                    breps.append(_scale_geometry_copy(brep, footing_to_member))
        else:
            base_plate = result.get("base_plate")
            if base_plate is not None:
                breps.append(_scale_geometry_copy(base_plate, footing_to_member))
            for brep in (result.get("plate_breps") or []):
                if brep is not None:
                    breps.append(_scale_geometry_copy(brep, footing_to_member))
        for key in ("stiffener_breps",):
            for brep in (result.get(key) or []):
                if brep is not None:
                    breps.append(_scale_geometry_copy(brep, footing_to_member))
    return breps, debug_records, handoff_records


def _cluster_key(point: Point3, tol: float) -> Tuple[int, int, int]:
    if tol <= 1e-12:
        tol = 1e-12
    return (
        int(round(point[0] / tol)),
        int(round(point[1] / tol)),
        int(round(point[2] / tol)),
    )


def _filter_support_members(
    members: Sequence[MemberRecord],
    support_z_tolerance: float = 1e-6,
    deduplicate_support_nodes: bool = True,
    support_node_merge_tolerance: float = 1e-3,
) -> Tuple[List[MemberRecord], Dict[str, Point3], Dict[str, List[MemberRecord]]]:
    support_members: List[MemberRecord] = []
    for member in members:
        base = _base_end_point(member)
        if abs(base[2]) <= support_z_tolerance:
            support_members.append(member)

    if not deduplicate_support_nodes:
        return support_members, {}, {member.member_id: [member] for member in support_members}

    clustered: Dict[Tuple[int, int, int], List[Tuple[MemberRecord, Point3]]] = {}
    for member in support_members:
        base = _base_end_point(member)
        key = _cluster_key(base, support_node_merge_tolerance)
        clustered.setdefault(key, []).append((member, base))

    deduped: List[MemberRecord] = []
    overrides: Dict[str, Point3] = {}
    cluster_members_by_representative: Dict[str, List[MemberRecord]] = {}
    for group in clustered.values():
        if not group:
            continue
        representative = min(group, key=lambda item: item[0].index)[0]
        n = float(len(group))
        cx = sum(p[0] for _, p in group) / n
        cy = sum(p[1] for _, p in group) / n
        cz = sum(p[2] for _, p in group) / n
        deduped.append(representative)
        overrides[representative.member_id] = (cx, cy, cz)
        cluster_members_by_representative[representative.member_id] = [
            member for member, _point in group
        ]

    return deduped, overrides, cluster_members_by_representative


def build_geometry_payload(
    line_model_path: Optional[Path] = None,
    plate_length: float = 0.8,
    plate_width: float = 0.8,
    plate_thickness: float = 0.02,
    bottom_face_mode: str = "Perpendicular_to_grain",
    include_hierarchies: Optional[Sequence[str]] = None,
    target_support_nodes_only: bool = False,
    support_z_tolerance: float = 1e-6,
    deduplicate_support_nodes: bool = True,
    support_node_merge_tolerance: float = 1e-3,
    target_member_index: Optional[int] = None,
    target_cluster_index: Optional[int] = None,
    geometry_kind: str = "flat_plate",
    geometry_units: str = "auto",
    sizing_recommendations: Optional[Dict[str, object]] = None,
    include_stiffeners: Optional[bool] = None,
    baseplate_top_z: Optional[float] = None,
    base_plate_brep: Optional[object] = None,
    base_diameter: Optional[float] = None,
    min_hole_edge_spacing: Optional[float] = None,
    heel_fillet_radius: Optional[float] = None,
    timber_bottom_gap: Optional[float] = None,
    min_timber_gap: Optional[float] = None,
    bottom_end_distance_multiplier: Optional[float] = None,
    webplate_thickness: Optional[float] = None,
    webplate_hole_diameter: Optional[float] = None,
    webplate_hole_pitch: Optional[float] = None,
    webplate_hole_transverse_spacing: Optional[float] = None,
    webplate_hole_rows: Optional[int] = None,
    webplate_hole_pattern: Optional[object] = None,
    webplate_hole_stagger_offset: Optional[float] = None,
    bolt_dia: Optional[float] = None,
    hole_clearance: Optional[float] = None,
    total_bolt_count: Optional[int] = None,
    stiffener_pair_axis_shift: Optional[float] = None,
    stiffener_pair_from_point: Optional[object] = None,
    stiffener_pair_to_point: Optional[object] = None,
) -> Dict[str, object]:
    bottom_face_mode = _canonical_bottom_face_mode(bottom_face_mode)
    requested_webplate_hole_mode = _coerce_int(webplate_hole_rows, None)
    if requested_webplate_hole_mode == 3 and webplate_hole_pattern is None:
        webplate_hole_pattern = "staggered_double_row"
        webplate_hole_rows = 2
    elif requested_webplate_hole_mode == 2 and webplate_hole_pattern is None:
        webplate_hole_pattern = "double_row"
    elif requested_webplate_hole_mode == 1 and webplate_hole_pattern is None:
        webplate_hole_pattern = "single_row_centerline"
    path = line_model_path or resolve_latest_line_model_path()
    members = load_member_records(path)
    source_member_count = len(members)
    source_geometry_units = _resolve_geometry_units(members, geometry_units)
    members = _scale_member_records(members, source_geometry_units, FOOTING_MODEL_UNITS)
    resolved_geometry_units = FOOTING_MODEL_UNITS

    if include_hierarchies:
        wanted = set(include_hierarchies)
        members = [m for m in members if m.hierarchy in wanted]

    center_overrides: Dict[str, Point3] = {}
    cluster_members_by_representative: Dict[str, List[MemberRecord]] = {}
    if target_support_nodes_only:
        members, center_overrides, cluster_members_by_representative = _filter_support_members(
            members,
            support_z_tolerance=support_z_tolerance,
            deduplicate_support_nodes=deduplicate_support_nodes,
            support_node_merge_tolerance=support_node_merge_tolerance,
        )

    target_member_index_mode: Optional[str] = None
    if target_member_index is not None:
        idx = int(target_member_index)
        selected = [m for m in members if m.index == idx]
        if selected:
            members = selected
            target_member_index_mode = "exact_index"
        elif 0 <= idx < len(members):
            # GH sliders often use local list positions (0..N-1); support that as fallback.
            members = [members[idx]]
            target_member_index_mode = "filtered_position"
        else:
            members = []
            target_member_index_mode = "not_found"
        if center_overrides:
            center_overrides = {
                member_id: center
                for member_id, center in center_overrides.items()
                if any(m.member_id == member_id for m in members)
            }
        if cluster_members_by_representative:
            cluster_members_by_representative = {
                member_id: cluster_members
                for member_id, cluster_members in cluster_members_by_representative.items()
                if any(m.member_id == member_id for m in members)
            }

    cluster_mode = None
    members, center_overrides, cluster_mode = _apply_target_cluster_filter(
        members,
        center_overrides,
        target_cluster_index,
    )
    if cluster_mode is not None:
        target_member_index_mode = cluster_mode
    if cluster_members_by_representative:
        selected_member_ids = {member.member_id for member in members}
        cluster_members_by_representative = {
            member_id: cluster_members
            for member_id, cluster_members in cluster_members_by_representative.items()
            if member_id in selected_member_ids
        }

    base_plates = build_base_plate_records(
        members=members,
        plate_length=plate_length,
        plate_width=plate_width,
        plate_thickness=plate_thickness,
        bottom_face_mode=bottom_face_mode,
        center_overrides=center_overrides,
    )

    geometry_kind_value = (geometry_kind or "flat_plate").strip().lower()
    resolved_sizing_recommendations = _extract_sizing_recommendations(sizing_recommendations)
    resolved_baseplate_top_z = (
        FOOTING_DEFAULT_BASEPLATE_TOP_Z
        if geometry_kind_value == "footing" and baseplate_top_z is None
        else baseplate_top_z
    )
    footing_breps: List[object] = []
    footing_debug: List[Dict[str, object]] = []
    footing_handoffs: List[Dict[str, object]] = []
    if geometry_kind_value == "footing":
        footing_breps, footing_debug, footing_handoffs = _build_footing_breps_for_members(
            members,
            center_overrides,
            cluster_members_by_representative=cluster_members_by_representative,
            sizing_recommendations=resolved_sizing_recommendations,
            bottom_face_mode=bottom_face_mode,
            include_stiffeners=include_stiffeners,
            baseplate_top_z=resolved_baseplate_top_z,
            base_plate_brep=base_plate_brep,
            base_diameter=base_diameter,
            min_hole_edge_spacing=min_hole_edge_spacing,
            heel_fillet_radius=heel_fillet_radius,
            timber_bottom_gap=timber_bottom_gap,
            min_timber_gap=min_timber_gap,
            bottom_end_distance_multiplier=bottom_end_distance_multiplier,
            webplate_thickness=webplate_thickness,
            webplate_hole_diameter=webplate_hole_diameter,
            webplate_hole_pitch=webplate_hole_pitch,
            webplate_hole_transverse_spacing=webplate_hole_transverse_spacing,
            webplate_hole_rows=webplate_hole_rows,
            webplate_hole_pattern=webplate_hole_pattern,
            webplate_hole_stagger_offset=webplate_hole_stagger_offset,
            bolt_dia=bolt_dia,
            hole_clearance=hole_clearance,
            total_bolt_count=total_bolt_count,
            stiffener_pair_axis_shift=stiffener_pair_axis_shift,
            stiffener_pair_from_point=stiffener_pair_from_point,
            stiffener_pair_to_point=stiffener_pair_to_point,
            geometry_units=resolved_geometry_units,
        )

    return to_payload_dict(
        members,
        base_plates,
        path,
        bottom_face_mode,
        source_member_count=source_member_count,
        target_support_nodes_only=target_support_nodes_only,
        deduplicate_support_nodes=deduplicate_support_nodes,
        target_member_index=target_member_index,
        target_member_index_mode=target_member_index_mode,
        target_cluster_index=target_cluster_index,
        geometry_kind=geometry_kind_value,
        geometry_units=resolved_geometry_units,
        source_geometry_units=source_geometry_units,
        footing_source_units=FOOTING_SOURCE_UNITS if geometry_kind_value == "footing" else None,
        footing_output_units=resolved_geometry_units if geometry_kind_value == "footing" else None,
        footing_sizing_source="engineering_sizing_recommendations"
        if resolved_sizing_recommendations
        else "code_baseline",
        footing_sizing_recommendations=resolved_sizing_recommendations,
        support_cluster_member_indices={
            member_id: [member.index for member in _incident_members_for_plate_slots(cluster_members)]
            for member_id, cluster_members in cluster_members_by_representative.items()
        },
        footing_breps=footing_breps,
        footing_debug=footing_debug,
        footing_handoffs=footing_handoffs,
    )


def build_preview_breps(base_plates: Sequence[BasePlateRecord]) -> List[object]:
    if rg is None:
        return []

    breps: List[object] = []
    for bp in base_plates:
        origin = rg.Point3d(*bp.center)
        xaxis = rg.Vector3d(*bp.x_axis)
        yaxis = rg.Vector3d(*bp.y_axis)
        plane = rg.Plane(origin, xaxis, yaxis)
        box = rg.Box(
            plane,
            rg.Interval(-0.5 * bp.length, 0.5 * bp.length),
            rg.Interval(-0.5 * bp.width, 0.5 * bp.width),
            rg.Interval(0.0, bp.thickness),
        )
        if not box.IsValid:
            continue
        brep = box.ToBrep()
        if brep is None:
            continue
        try:
            if hasattr(brep, "IsValid") and not brep.IsValid:
                continue
        except Exception:
            pass
        breps.append(brep)
    return breps


def _cli() -> None:
    payload = build_geometry_payload()
    print(json.dumps(payload["metadata"], indent=2))


if __name__ == "__main__":
    _cli()


# --- Inlined base_footing_component.py ---
"""
Grasshopper-facing helper for parametrizing the T2 canopy base footing plates.

The helper can either extract plate/hole parameters from referenced Rhino
geometry or rebuild the geometry from explicit GH inputs. Geometry creation is
kept in Rhino.Geometry so the file stays paste/import friendly for Rhino 8 Py3.
"""

import math

try:
    import Rhino.Geometry as rg
except Exception:
    rg = None


SOURCE_NOTE = "2026-05_Base_Footing_T2Canopy.3dm"
SOURCE_UNITS = FOOTING_MODEL_UNITS
FOOTING_BOOLEAN_TOLERANCE = 0.01 * _unit_scale_factor(FOOTING_SOURCE_UNITS, SOURCE_UNITS)
FOOTING_PLATE_HOLE_PLUG_MARGIN = 0.05 * _unit_scale_factor(FOOTING_SOURCE_UNITS, SOURCE_UNITS)
FOOTING_MIN_HOLE_CUTTER_DEPTH = 1.0 * _unit_scale_factor(FOOTING_SOURCE_UNITS, SOURCE_UNITS)
FOOTING_BOOLEAN_OVERLAP = 0.5 * _unit_scale_factor(FOOTING_SOURCE_UNITS, SOURCE_UNITS)
FOOTING_HOLE_CUTTER_THICKNESS_FACTOR = 8.0
FOOTING_HOLE_CUTTER_DIAMETER_FACTOR = 4.0
FOOTING_DEFAULT_BASEPLATE_TOP_Z = 0.3658

SOURCE_BASE_CENTER = (95980.17681639228, 13550.621533456946, 0.0)
SOURCE_WEB_PLATE_CENTER = (96044.070, 12754.825, 0.0)
SOURCE_FULL_PLATE_CENTER = (95060.8525, 11651.119, 0.0)
SOURCE_STIFFENER_CENTER = (96491.2035, 11874.865, 0.0)

REFERENCE = {
    "base_shape": "circular",
    "base_diameter": 700.0,
    "base_length": 700.0,
    "base_width": 700.0,
    "base_thickness": 20.0,
    "hole_diameter": 50.0,
    "hole_spacing": (295.393, 292.9625),
    "edge_spacing": (40.0, 40.0, 40.0, 40.0),
    "omit_center_hole": True,
    "plates": [
        {
            "source_index": 20,
            "center": (95993.783, 12882.555, 301.227),
            "length": 287.302,
            "width": 140.0,
            "thickness": 10.0,
            "support_heel_local": (-80.282, 70.0, 0.001),
            "azimuth_deg": 111.801,
            "inclination_deg": 60.608,
            "edge_lengths": (10.0, 140.0, 163.520, 202.810, 287.302),
        },
        {
            "source_index": 21,
            "center": (95992.868, 12625.737, 300.282),
            "length": 287.402,
            "width": 140.0,
            "thickness": 10.0,
            "support_heel_local": (-78.962, 70.001, 0.0),
            "azimuth_deg": -111.801,
            "inclination_deg": 60.081,
            "edge_lengths": (10.0, 140.0, 164.472, 201.083, 287.402),
        },
        {
            "source_index": 22,
            "center": (96093.151, 12876.445, 303.644),
            "length": 286.967,
            "width": 140.0,
            "thickness": 10.0,
            "support_heel_local": (-83.805, 70.001, 0.0),
            "azimuth_deg": 68.199,
            "inclination_deg": 62.055,
            "edge_lengths": (10.0, 140.0, 161.032, 207.397, 286.967),
        },
        {
            "source_index": 23,
            "center": (96093.745, 12632.649, 303.055),
            "length": 287.053,
            "width": 140.0,
            "thickness": 10.0,
            "support_heel_local": (-82.997, 70.0, 0.0),
            "azimuth_deg": -68.199,
            "inclination_deg": 61.718,
            "edge_lengths": (10.0, 140.0, 161.596, 206.348, 287.053),
        },
    ],
}

FULL_PLATE_REFERENCE = [
    {
        "source_index": 30,
        "center": (95133.200, 11824.678, 402.527),
        "full_length": 620.138,
        "straight_length": 287.302,
        "width": 139.219,
        "thickness": 10.0,
        "azimuth_deg": 66.492,
        "inclination_deg": 60.625,
    },
    {
        "source_index": 31,
        "center": (94988.505, 11824.678, 402.527),
        "full_length": 620.138,
        "straight_length": 287.402,
        "width": 139.219,
        "thickness": 10.0,
        "azimuth_deg": 113.508,
        "inclination_deg": 60.625,
    },
    {
        "source_index": 32,
        "center": (95133.200, 11477.560, 402.527),
        "full_length": 620.138,
        "straight_length": 286.967,
        "width": 139.219,
        "thickness": 10.0,
        "azimuth_deg": -66.492,
        "inclination_deg": 60.625,
    },
    {
        "source_index": 33,
        "center": (94988.505, 11477.560, 402.527),
        "full_length": 620.138,
        "straight_length": 287.053,
        "width": 139.219,
        "thickness": 10.0,
        "azimuth_deg": -113.508,
        "inclination_deg": 60.625,
    },
]

STIFFENER_REFERENCE = [
    {
        "source_faces": (110, 115),
        "center": (96513.898, 11817.917, 203.710),
        "length": 100.407,
        "height": 285.653,
        "low_height": 141.613,
        "thickness": 10.0,
        "azimuth_deg": -40.427,
        "profile_points": (
            (-50.104, -106.997),
            (50.125, -106.997),
            (50.162, 178.655),
            (-50.245, 34.616),
        ),
    },
    {
        "source_faces": (51, 56),
        "center": (96468.509, 11817.917, 203.710),
        "length": 100.407,
        "height": 285.653,
        "low_height": 141.613,
        "thickness": 10.0,
        "azimuth_deg": 40.427,
        "profile_points": (
            (-50.125, -106.997),
            (50.104, -106.997),
            (50.245, 34.616),
            (-50.162, 178.655),
        ),
    },
    {
        "source_faces": (111, 116),
        "center": (96513.883, 11931.813, 203.710),
        "length": 100.342,
        "height": 285.653,
        "low_height": 141.613,
        "thickness": 10.0,
        "azimuth_deg": -139.573,
        "profile_points": (
            (-50.129, -106.997),
            (50.100, -106.997),
            (50.213, 34.616),
            (-50.129, 178.655),
        ),
    },
    {
        "source_faces": (52, 57),
        "center": (96468.524, 11931.813, 203.710),
        "length": 100.342,
        "height": 285.653,
        "low_height": 141.613,
        "thickness": 10.0,
        "azimuth_deg": 139.573,
        "profile_points": (
            (-50.100, -106.997),
            (50.129, -106.997),
            (50.129, 178.655),
            (-50.213, 34.616),
        ),
    },
]

PLATE_HOLE_REFERENCE = [
    {
        "source_index": 30,
        "source_face_indices": (6, 7, 8, 9),
        "center": (95164.069, 11897.903, 509.552),
        "row_count": 1,
        "holes_per_row": 4,
        "diameter": 37.125,
        "row_spacing": 0.0,
        "pitch": 74.25,
    },
    {
        "source_index": 31,
        "source_face_indices": (6, 7, 8, 9),
        "center": (94957.636, 11897.903, 509.552),
        "row_count": 1,
        "holes_per_row": 4,
        "diameter": 37.125,
        "row_spacing": 0.0,
        "pitch": 74.25,
    },
    {
        "source_index": 32,
        "source_face_indices": (6, 7, 8, 9),
        "center": (95164.069, 11404.335, 509.552),
        "row_count": 1,
        "holes_per_row": 4,
        "diameter": 37.125,
        "row_spacing": 0.0,
        "pitch": 74.25,
    },
    {
        "source_index": 33,
        "source_face_indices": (6, 7, 8, 9),
        "center": (94957.636, 11404.335, 509.552),
        "row_count": 1,
        "holes_per_row": 4,
        "diameter": 37.125,
        "row_spacing": 0.0,
        "pitch": 74.25,
    },
]

_scale_footing_reference_data_to_meters()


def _flatten_values(value, _seen=None):
    if value is None:
        return []
    if isinstance(value, (str, bytes)):
        return [value]

    if _seen is None:
        _seen = set()
    marker = id(value)
    if marker in _seen:
        return []
    _seen.add(marker)

    branch_count = getattr(value, "BranchCount", None)
    branch_getter = getattr(value, "Branch", None)
    if isinstance(branch_count, int) and callable(branch_getter):
        items = []
        for index in range(branch_count):
            try:
                items.extend(_flatten_values(branch_getter(index), _seen))
            except Exception:
                pass
        return items

    try:
        return list(value)
    except TypeError:
        return [value]


def _coerce_float(value, default=None):
    if value is None:
        return default
    try:
        return float(value)
    except Exception:
        return default


def _coerce_bool(value, default=False):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "y", "on"):
        return True
    if text in ("0", "false", "no", "n", "off"):
        return False
    return default


def _coerce_optional_bool(value):
    if value is None:
        return None
    return _coerce_bool(value, None)


def _has_values(value):
    return bool(_flatten_values(value))


def _list_at(values, index, default=None):
    items = _flatten_values(values)
    if not items:
        return default
    if len(items) == 1:
        return items[0]
    if index < len(items):
        return items[index]
    return items[-1]


def _float_at(values, index, default=None):
    return _coerce_float(_list_at(values, index, default), default)


def _coerce_int(value, default=None):
    if value is None:
        return default
    try:
        return int(value)
    except Exception:
        return default


def _int_at(values, index, default=None):
    return _coerce_int(_list_at(values, index, default), default)


def _point_from_value(value, default=None):
    if value is None:
        return default
    if rg is not None and hasattr(value, "X") and hasattr(value, "Y") and hasattr(value, "Z"):
        return rg.Point3d(float(value.X), float(value.Y), float(value.Z))
    try:
        items = list(value)
    except Exception:
        return default
    if len(items) < 3:
        return default
    if rg is None:
        return (float(items[0]), float(items[1]), float(items[2]))
    return rg.Point3d(float(items[0]), float(items[1]), float(items[2]))


def _default_plane():
    if rg is None:
        return None
    return rg.Plane(
        rg.Point3d(*SOURCE_BASE_CENTER),
        rg.Vector3d.XAxis,
        rg.Vector3d.YAxis,
    )


def _plane_from_outline(reference_outline):
    if rg is None or reference_outline is None:
        return None
    outline = _list_at(reference_outline, 0, reference_outline)
    if outline is None or not hasattr(outline, "GetBoundingBox"):
        return None

    bbox = outline.GetBoundingBox(True)
    center = bbox.Center
    try_get_plane = getattr(outline, "TryGetPlane", None)
    if callable(try_get_plane):
        try:
            result = try_get_plane()
            if isinstance(result, tuple):
                ok, plane = result
                if ok:
                    return rg.Plane(center, plane.XAxis, plane.YAxis)
        except Exception:
            pass
    return rg.Plane(center, rg.Vector3d.XAxis, rg.Vector3d.YAxis)


def _as_plane(value, reference_outline=None):
    if rg is None:
        return None
    if value is not None and hasattr(value, "Origin") and hasattr(value, "XAxis") and hasattr(value, "YAxis"):
        return value
    outline_plane = _plane_from_outline(reference_outline)
    if outline_plane is not None:
        return outline_plane
    return _default_plane()


def _reference_local_point(world_xyz):
    return (
        float(world_xyz[0]) - SOURCE_BASE_CENTER[0],
        float(world_xyz[1]) - SOURCE_BASE_CENTER[1],
        float(world_xyz[2]) - SOURCE_BASE_CENTER[2],
    )


def _point_on_plane(plane, local_xyz):
    if rg is None:
        return local_xyz
    point = rg.Point3d(plane.Origin)
    point += plane.XAxis * float(local_xyz[0])
    point += plane.YAxis * float(local_xyz[1])
    point += plane.ZAxis * float(local_xyz[2])
    return point


def _vector_on_plane(plane, local_xyz):
    if rg is None or plane is None:
        return None
    vector = rg.Vector3d(0.0, 0.0, 0.0)
    vector += plane.XAxis * float(local_xyz[0])
    vector += plane.YAxis * float(local_xyz[1])
    vector += plane.ZAxis * float(local_xyz[2])
    if vector.IsTiny():
        return None
    vector.Unitize()
    return vector


def _vector_from_angles(azimuth_deg, inclination_deg):
    azimuth = math.radians(float(azimuth_deg))
    inclination = math.radians(float(inclination_deg))
    x = math.cos(inclination) * math.cos(azimuth)
    y = math.cos(inclination) * math.sin(azimuth)
    z = math.sin(inclination)
    if rg is None:
        return (x, y, z)
    vector = rg.Vector3d(x, y, z)
    vector.Unitize()
    return vector


def _line_like_endpoints(value):
    if value is None:
        return None

    if hasattr(value, "PointAtStart") and hasattr(value, "PointAtEnd"):
        return value.PointAtStart, value.PointAtEnd
    if hasattr(value, "From") and hasattr(value, "To"):
        return value.From, value.To
    if hasattr(value, "FromX") and hasattr(value, "ToX"):
        if rg is None:
            return None
        return (
            rg.Point3d(value.FromX, value.FromY, value.FromZ),
            rg.Point3d(value.ToX, value.ToY, value.ToZ),
        )
    return None


def _altitude_from_line_like(value):
    endpoints = _line_like_endpoints(value)
    if endpoints is None:
        return None
    start, end = endpoints
    dx = float(end.X - start.X)
    dy = float(end.Y - start.Y)
    dz = float(end.Z - start.Z)
    horizontal = math.sqrt(dx * dx + dy * dy)
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        return None
    return math.degrees(math.atan2(abs(dz), horizontal))


def _timber_altitude_at(timber_elements, plate_timber_indices, plate_index):
    elements = _flatten_values(timber_elements)
    if not elements:
        return None

    default_index = plate_index if plate_index < len(elements) else None
    timber_index = _int_at(plate_timber_indices, plate_index, default_index)
    if timber_index is None or timber_index < 0 or timber_index >= len(elements):
        return None
    return _altitude_from_line_like(elements[timber_index])


def _plate_axes(azimuth_deg, inclination_deg):
    if rg is None:
        return None, None, None
    long_axis = _vector_from_angles(azimuth_deg, inclination_deg)
    azimuth = math.radians(float(azimuth_deg))
    thickness_axis = rg.Vector3d(-math.sin(azimuth), math.cos(azimuth), 0.0)
    if not thickness_axis.Unitize():
        thickness_axis = rg.Vector3d.YAxis
    width_axis = rg.Vector3d.CrossProduct(thickness_axis, long_axis)
    if not width_axis.Unitize():
        width_axis = rg.Vector3d.ZAxis
    return long_axis, width_axis, thickness_axis


def _plate_plane(center, azimuth_deg, inclination_deg):
    if rg is None:
        return None
    long_axis, width_axis, _thickness_axis = _plate_axes(azimuth_deg, inclination_deg)
    return rg.Plane(center, long_axis, width_axis)


def _horizontal_plane(center, azimuth_deg):
    if rg is None:
        return None
    azimuth = math.radians(float(azimuth_deg))
    x_axis = rg.Vector3d(math.cos(azimuth), math.sin(azimuth), 0.0)
    if not x_axis.Unitize():
        x_axis = rg.Vector3d.XAxis
    y_axis = rg.Vector3d(-math.sin(azimuth), math.cos(azimuth), 0.0)
    if not y_axis.Unitize():
        y_axis = rg.Vector3d.YAxis
    return rg.Plane(center, x_axis, y_axis)


def _duplicate_geometry(geometry):
    if geometry is None:
        return None
    duplicate = getattr(geometry, "Duplicate", None)
    if callable(duplicate):
        return duplicate()
    duplicate_brep = getattr(geometry, "DuplicateBrep", None)
    if callable(duplicate_brep):
        return duplicate_brep()
    return None


def _to_brep(geometry):
    if geometry is None:
        return None
    if hasattr(geometry, "Edges") and hasattr(geometry, "Faces"):
        return geometry
    to_brep = getattr(geometry, "ToBrep", None)
    if callable(to_brep):
        try:
            return to_brep(False)
        except TypeError:
            try:
                return to_brep()
            except Exception:
                return None
        except Exception:
            return None
    return None


def _transform_between_planes(geometry, source_plane, target_plane, scale_x=1.0, scale_y=1.0, scale_z=1.0):
    if rg is None or geometry is None or source_plane is None or target_plane is None:
        return None
    transformed = _duplicate_geometry(geometry)
    if transformed is None:
        transformed = _to_brep(geometry)
        transformed = _duplicate_geometry(transformed)
    if transformed is None:
        return None

    try:
        transformed.Transform(rg.Transform.PlaneToPlane(source_plane, rg.Plane.WorldXY))
        transformed.Transform(rg.Transform.Scale(rg.Plane.WorldXY, scale_x, scale_y, scale_z))
        transformed.Transform(rg.Transform.PlaneToPlane(rg.Plane.WorldXY, target_plane))
    except Exception:
        return None
    return transformed


def _edge_spacing_tuple(value):
    items = [_coerce_float(item) for item in _flatten_values(value)]
    items = [item for item in items if item is not None]
    if not items:
        return tuple(REFERENCE["edge_spacing"])
    if len(items) == 1:
        return (items[0], items[0], items[0], items[0])
    if len(items) == 2:
        return (items[0], items[0], items[1], items[1])
    return (items[0], items[1], items[2], items[3])


def _spacing_pair(value):
    items = [_coerce_float(item) for item in _flatten_values(value)]
    items = [item for item in items if item is not None]
    if not items:
        return tuple(REFERENCE["hole_spacing"])
    if len(items) == 1:
        return (items[0], items[0])
    return (items[0], items[1])


def _row_count_at(values, index, default=1):
    value = _int_at(values, index, default)
    return 2 if value in (2, 3) else 1


def _plate_hole_pattern_at(values, index, default=""):
    value = _list_at(_flatten_values(values), index, None)
    pattern = _canonical_plate_hole_pattern(value)
    return pattern or _canonical_plate_hole_pattern(default)


def _unique_lengths(lengths, tolerance=0.5):
    unique = []
    for length in sorted(lengths):
        if length <= 1e-6:
            continue
        if not unique or abs(length - unique[-1]) > tolerance:
            unique.append(length)
    return unique


def _edge_vector(edge):
    try:
        p0 = edge.PointAtStart
        p1 = edge.PointAtEnd
    except Exception:
        return None, None, None
    dx = p1.X - p0.X
    dy = p1.Y - p0.Y
    dz = p1.Z - p0.Z
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        return None, None, None
    return length, (dx, dy, dz), (p0, p1)


def _support_heel_local_from_brep(brep, center, azimuth_deg, inclination_deg):
    if rg is None or brep is None or center is None:
        return None
    vertices = getattr(brep, "Vertices", None)
    if not vertices:
        return None
    points = [vertex.Location for vertex in vertices]
    if len(points) < 2:
        return None

    z_groups = _cluster_values([point.Z for point in points], tolerance=max(FOOTING_BOOLEAN_TOLERANCE, 1e-6))
    if len(z_groups) < 2:
        return None
    heel_z = _median(z_groups[1])
    heel_points = [
        point
        for point in points
        if abs(point.Z - heel_z) <= max(FOOTING_BOOLEAN_TOLERANCE, 1e-6)
    ]
    if not heel_points:
        return None

    heel_center = rg.Point3d(
        sum(point.X for point in heel_points) / len(heel_points),
        sum(point.Y for point in heel_points) / len(heel_points),
        sum(point.Z for point in heel_points) / len(heel_points),
    )
    plane = _plate_plane(center, azimuth_deg, inclination_deg)
    return _point_local_in_plane(heel_center, plane)


def extract_plate_specs(reference_plates):
    specs = []
    for index, brep in enumerate(_flatten_values(reference_plates)):
        if brep is None or not hasattr(brep, "Edges"):
            continue

        edge_records = []
        for edge in brep.Edges:
            length, vector, _points = _edge_vector(edge)
            if length is not None:
                edge_records.append((length, vector))
        if not edge_records:
            continue

        lengths = _unique_lengths([record[0] for record in edge_records])
        thickness = lengths[0] if lengths else 0.0
        width = lengths[1] if len(lengths) > 1 else 0.0
        length = lengths[-1] if lengths else 0.0

        long_record = max(edge_records, key=lambda item: item[0])
        lx, ly, lz = long_record[1]
        if lz < 0:
            lx, ly, lz = -lx, -ly, -lz
        azimuth = math.degrees(math.atan2(ly, lx))
        inclination = math.degrees(math.atan2(lz, math.sqrt(lx * lx + ly * ly)))

        vertices = getattr(brep, "Vertices", None)
        center = None
        if vertices:
            points = [vertex.Location for vertex in vertices]
            if points:
                center = rg.Point3d(
                    sum(point.X for point in points) / len(points),
                    sum(point.Y for point in points) / len(points),
                    sum(point.Z for point in points) / len(points),
                )
        if center is None:
            bbox = brep.GetBoundingBox(True)
            center = bbox.Center
        support_heel_local = _support_heel_local_from_brep(
            brep,
            center,
            azimuth,
            inclination,
        )

        specs.append(
            {
                "source_index": index,
                "source_center": center,
                "center": center,
                "length": length,
                "width": width,
                "thickness": thickness,
                "azimuth_deg": azimuth,
                "altitude_deg": inclination,
                "inclination_deg": inclination,
                "support_heel_local": support_heel_local,
                "edge_lengths": tuple(lengths),
            }
        )
    return specs


def extract_oriented_brep_specs(reference_geometry, target_width=140.0, target_thickness=10.0):
    specs = []
    for index, geometry in enumerate(_flatten_values(reference_geometry)):
        brep = _to_brep(geometry)
        if brep is None or not hasattr(brep, "Edges"):
            continue

        edge_records = []
        for edge in brep.Edges:
            length, vector, _points = _edge_vector(edge)
            if length is not None:
                edge_records.append((length, vector))
        if not edge_records:
            continue

        long_record = max(edge_records, key=lambda item: item[0])
        thick_record = min(edge_records, key=lambda item: abs(item[0] - target_thickness))
        width_record = min(edge_records, key=lambda item: abs(item[0] - target_width))

        lx, ly, lz = long_record[1]
        if lz < 0:
            lx, ly, lz = -lx, -ly, -lz
        long_axis = rg.Vector3d(lx, ly, lz)
        long_axis.Unitize()

        tx, ty, tz = thick_record[1]
        thickness_axis = rg.Vector3d(tx, ty, tz)
        dot = (
            thickness_axis.X * long_axis.X
            + thickness_axis.Y * long_axis.Y
            + thickness_axis.Z * long_axis.Z
        )
        if dot < 0:
            thickness_axis.Reverse()
        if not thickness_axis.Unitize():
            _long_axis, _width_axis, thickness_axis = _plate_axes(
                math.degrees(math.atan2(ly, lx)),
                math.degrees(math.atan2(lz, math.sqrt(lx * lx + ly * ly))),
            )

        width_axis = rg.Vector3d.CrossProduct(thickness_axis, long_axis)
        if not width_axis.Unitize():
            wx, wy, wz = width_record[1]
            width_axis = rg.Vector3d(wx, wy, wz)
            width_axis.Unitize()

        vertices = getattr(brep, "Vertices", None)
        if vertices:
            points = [vertex.Location for vertex in vertices]
            center = rg.Point3d(
                sum(point.X for point in points) / len(points),
                sum(point.Y for point in points) / len(points),
                sum(point.Z for point in points) / len(points),
            )
        else:
            center = brep.GetBoundingBox(True).Center

        azimuth = math.degrees(math.atan2(long_axis.Y, long_axis.X))
        inclination = math.degrees(math.atan2(long_axis.Z, math.sqrt(long_axis.X * long_axis.X + long_axis.Y * long_axis.Y)))
        specs.append(
            {
                "source_index": index,
                "center": center,
                "length": long_record[0],
                "width": width_record[0],
                "thickness": thick_record[0],
                "azimuth_deg": azimuth,
                "altitude_deg": inclination,
                "inclination_deg": inclination,
                "plane": rg.Plane(center, long_axis, width_axis),
                "edge_lengths": tuple(_unique_lengths([record[0] for record in edge_records])),
            }
        )
    return specs


def extract_hole_specs(reference_hole_curves):
    specs = []
    for index, curve in enumerate(_flatten_values(reference_hole_curves)):
        if curve is None or not hasattr(curve, "GetBoundingBox"):
            continue
        bbox = curve.GetBoundingBox(True)
        min_pt = bbox.Min
        max_pt = bbox.Max
        center = rg.Point3d(
            0.5 * (min_pt.X + max_pt.X),
            0.5 * (min_pt.Y + max_pt.Y),
            0.5 * (min_pt.Z + max_pt.Z),
        )
        diameter = max(max_pt.X - min_pt.X, max_pt.Y - min_pt.Y)
        specs.append({"source_index": index, "center": center, "diameter": diameter})
    return specs


def _default_plate_specs(plane):
    specs = []
    for spec in REFERENCE["plates"]:
        local = (
            float(spec["center"][0]) - SOURCE_WEB_PLATE_CENTER[0],
            float(spec["center"][1]) - SOURCE_WEB_PLATE_CENTER[1],
            float(spec["center"][2]) - SOURCE_WEB_PLATE_CENTER[2],
        )
        next_spec = dict(spec)
        next_spec["source_center"] = _point_from_value(spec["center"], spec["center"])
        next_spec["center"] = _point_on_plane(plane, local)
        specs.append(next_spec)
    return specs


def _default_full_plate_specs(plane):
    specs = []
    for spec in FULL_PLATE_REFERENCE:
        local = (
            spec["center"][0] - SOURCE_FULL_PLATE_CENTER[0],
            spec["center"][1] - SOURCE_FULL_PLATE_CENTER[1],
            spec["center"][2] - SOURCE_FULL_PLATE_CENTER[2],
        )
        next_spec = dict(spec)
        next_spec["center"] = _point_on_plane(plane, local)
        next_spec["length"] = spec["full_length"]
        next_spec["plane"] = _plate_plane(
            next_spec["center"],
            spec["azimuth_deg"],
            spec["inclination_deg"],
        )
        specs.append(next_spec)
    return specs


def _default_stiffener_specs(plane):
    specs = []
    for spec in STIFFENER_REFERENCE:
        local = (
            spec["center"][0] - SOURCE_STIFFENER_CENTER[0],
            spec["center"][1] - SOURCE_STIFFENER_CENTER[1],
            spec["center"][2] - SOURCE_STIFFENER_CENTER[2],
        )
        next_spec = dict(spec)
        next_spec["center"] = _point_on_plane(plane, local)
        specs.append(next_spec)
    return specs


def _rotate_xy(local_xy, angle_deg):
    angle = math.radians(float(angle_deg))
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return (
        local_xy[0] * cos_a - local_xy[1] * sin_a,
        local_xy[0] * sin_a + local_xy[1] * cos_a,
    )


def _stiffener_specs_from_plate_targets(
    baseplate_top_plane,
    plate_specs,
    plate_timber_widths=None,
    plate_timber_heights=None,
    bottom_face_mode="Perpendicular_to_grain",
    default_thickness=None,
    stiffener_pair_axis_shift=None,
    stiffener_pair_from_point=None,
    stiffener_pair_to_point=None,
):
    """Build two target-driven stiffener plates for every live webplate.

    The legacy Rhino references contained one triangular proxy per quadrant.
    The footing detail actually needs a pair per webplate:
    1) a plate on the timber bottom face, and
    2) a vertical plate that rises from the baseplate and intersects it.
    """
    if rg is None or baseplate_top_plane is None or not plate_specs:
        return []

    specs = []
    mode = _canonical_bottom_face_mode(bottom_face_mode)
    baseplate_top_z = float(baseplate_top_plane.Origin.Z)
    timber_width_inputs = _flatten_values(plate_timber_widths)
    timber_height_inputs = _flatten_values(plate_timber_heights)
    explicit_pair_axis_shift = _coerce_float(stiffener_pair_axis_shift, None)
    pair_shift_from_point = _point_from_value(stiffener_pair_from_point)
    pair_shift_to_point = _point_from_value(stiffener_pair_to_point)
    pair_shift_vector_input = None
    if (
        rg is not None
        and pair_shift_from_point is not None
        and pair_shift_to_point is not None
    ):
        pair_shift_vector_input = pair_shift_to_point - pair_shift_from_point

    for index, plate_spec in enumerate(plate_specs):
        plate_plane = _plate_plane(
            plate_spec["center"],
            plate_spec["azimuth_deg"],
            plate_spec["inclination_deg"],
        )
        heel = _plate_support_heel_point(plate_spec)
        if plate_plane is None or heel is None:
            continue

        pair_axis_shift = explicit_pair_axis_shift
        pair_axis_shift_source = "none"
        if pair_axis_shift is not None:
            pair_axis_shift_source = "axis_shift_input"
        elif pair_shift_vector_input is not None:
            pair_axis_shift = float(
                pair_shift_vector_input.X * plate_plane.XAxis.X
                + pair_shift_vector_input.Y * plate_plane.XAxis.Y
                + pair_shift_vector_input.Z * plate_plane.XAxis.Z
            )
            pair_axis_shift_source = "point_pair_projected_to_webplate_axis"
        else:
            pair_axis_shift = 0.0

        pair_axis_translation = rg.Vector3d(plate_plane.XAxis)
        if not pair_axis_translation.Unitize():
            pair_axis_translation = rg.Vector3d(0.0, 0.0, 0.0)
            pair_axis_shift = 0.0
            if pair_axis_shift_source != "none":
                pair_axis_shift_source = "invalid_webplate_axis"
        else:
            pair_axis_translation *= float(pair_axis_shift or 0.0)

        collision_axis_distance = _coerce_float(
            plate_spec.get("collision_axis_distance"),
            0.0,
        ) or 0.0
        heel_local = _point_local_in_plane(heel, plate_plane)
        collision_centerline_point = _point_on_plane(
            plate_plane,
            (
                heel_local[0] + collision_axis_distance,
                0.0,
                0.0,
            ),
        )
        effective_timber_bottom_z = _coerce_float(
            plate_spec.get("effective_timber_bottom_z"),
            collision_centerline_point.Z,
        )
        if effective_timber_bottom_z is not None:
            collision_centerline_point.Z = float(effective_timber_bottom_z)

        timber_width = _float_at(
            timber_width_inputs,
            index,
            plate_spec.get("timber_width", plate_spec.get("width")),
        )
        timber_width = max(float(timber_width or plate_spec.get("width") or 0.0), 1e-6)
        timber_height = _float_at(
            timber_height_inputs,
            index,
            plate_spec.get("timber_height", plate_spec.get("width")),
        )
        timber_height = max(float(timber_height or plate_spec.get("width") or timber_width), 1e-6)
        webplate_depth = max(float(plate_spec.get("width") or timber_height), 1e-6)
        thickness = max(
            float(
                _coerce_float(
                    default_thickness,
                    plate_spec.get("thickness", 0.010),
                )
                or 0.010
            ),
            1e-6,
        )

        width_axis = rg.Vector3d(plate_plane.ZAxis)
        if not width_axis.Unitize():
            width_axis = rg.Vector3d.YAxis
        if mode == "Parallel_to_ground":
            face_axis = rg.Vector3d(
                plate_plane.XAxis.X,
                plate_plane.XAxis.Y,
                0.0,
            )
            if not face_axis.Unitize():
                face_axis = rg.Vector3d.XAxis
            # Keep the same width-edge hinge, but swing the horizontal face to
            # the opposite side of that hinge from the first implementation.
            face_axis.Reverse()
        else:
            face_axis = rg.Vector3d(plate_plane.YAxis)
            if not face_axis.Unitize():
                face_axis = rg.Vector3d.ZAxis

        # The webplate stays centered on the timber member axis. One stiffener
        # pair spans the full timber width, so it works on both sides of the
        # centered webplate. In side view, the timber-face plate runs from the
        # heel-side tip on the webplate to the opposite depth edge.
        support_toe_local = plate_spec.get("support_toe_local") or (
            -0.5 * float(plate_spec.get("length") or 0.0),
            -0.5 * float(plate_spec.get("width") or 0.0),
            0.0,
        )
        support_heel_local = plate_spec.get("support_heel_local") or (
            -0.5 * float(plate_spec.get("length") or 0.0),
            0.5 * float(plate_spec.get("width") or 0.0),
            0.0,
        )
        reference_width = float(
            plate_spec.get("reference_width")
            or plate_spec.get("width")
            or 1.0
        )
        toe_y = (
            float(support_toe_local[1])
            * float(plate_spec.get("width") or reference_width)
            / reference_width
        )
        heel_y = (
            float(support_heel_local[1])
            * float(plate_spec.get("width") or reference_width)
            / reference_width
        )
        # The pair translates from its shared-edge midpoint to the midpoint on
        # the selected side face where the timber bottom-face plane crosses it.
        # The earlier projection only chooses the side-face rail to follow.
        edge_projected_collision_point = plate_spec.get("edge_projected_collision_point")
        edge_projected_collision_name = plate_spec.get("edge_projected_collision_name")
        projected_collision_point = plate_spec.get("projected_collision_point")

        bottom_face_center_shift = 0.0
        pivot_data = _bottom_face_pivot_data(
            plate_spec,
            plate_plane,
            collision_centerline_point,
            effective_timber_bottom_z,
        )
        bottom_face_anchor = pivot_data.get("point") or collision_centerline_point
        bottom_face_anchor_source = pivot_data.get("source") or "collision_centerline_fallback"
        # The bottom-face plane pivots about the timber-width edge, not through
        # the center of the timber end face. Keep the stiffener rectangle
        # centered by backing the local origin off from that edge anchor.
        bottom_face_center = rg.Point3d(bottom_face_anchor)
        bottom_face_center -= face_axis * heel_y
        timber_face_dividing_plane = rg.Plane(bottom_face_center, width_axis, face_axis)
        heel_face_midline_start, heel_face_midline_end = _plate_selected_face_midline_segment(
            plate_spec,
            plate_plane,
            edge_projected_collision_name,
        )
        heel_face_midpoint_at_timber_face, heel_face_intersection_param = _line_plane_intersection_point(
            heel_face_midline_start,
            heel_face_midline_end,
            timber_face_dividing_plane,
        )
        if heel_face_midpoint_at_timber_face is not None:
            webplate_intersection_target = rg.Point3d(heel_face_midpoint_at_timber_face)
            webplate_intersection_source = "heel_face_midpoint_at_timber_face"
        elif edge_projected_collision_point is not None:
            webplate_intersection_target = rg.Point3d(edge_projected_collision_point)
            webplate_intersection_source = "nearest_webplate_edge_projection_fallback"
        elif projected_collision_point is not None:
            webplate_intersection_target = rg.Point3d(projected_collision_point)
            webplate_intersection_source = "webplate_plane_projection_fallback"
        else:
            webplate_intersection_target = rg.Point3d(collision_centerline_point)
            webplate_intersection_source = "heel_axis_fallback"
        # The actual translation datum is midpoint-to-midpoint:
        # 1) the midpoint of the shared edge between the two stiffeners
        # 2) the midpoint where the timber bottom face meets the heel face.
        #
        # The shared edge is not the timber-face plate center. It is the heel-side
        # depth edge where the vertical stiffener intersects the timber-face plate.
        unsnapped_pair_interface_tip = _point_on_plane(
            timber_face_dividing_plane,
            (0.0, heel_y, 0.0),
        )
        unsnapped_pair_shared_edge_midpoint = rg.Point3d(unsnapped_pair_interface_tip)
        heel_edge_centerline_point = rg.Point3d(unsnapped_pair_shared_edge_midpoint)
        timber_bottom_face_heel_intersection_point = rg.Point3d(webplate_intersection_target)
        pair_snap_vector = (
            timber_bottom_face_heel_intersection_point
            - unsnapped_pair_shared_edge_midpoint
        )
        snapped_timber_face_dividing_origin = rg.Point3d(timber_face_dividing_plane.Origin)
        snapped_timber_face_dividing_origin += pair_snap_vector
        snapped_timber_face_dividing_origin += pair_axis_translation
        snapped_timber_face_dividing_plane = rg.Plane(
            snapped_timber_face_dividing_origin,
            timber_face_dividing_plane.XAxis,
            timber_face_dividing_plane.YAxis,
        )
        shifted_webplate_intersection_target = rg.Point3d(webplate_intersection_target)
        shifted_webplate_intersection_target += pair_axis_translation
        snapped_pair_shared_edge_midpoint = rg.Point3d(unsnapped_pair_shared_edge_midpoint)
        snapped_pair_shared_edge_midpoint += pair_snap_vector
        snapped_pair_shared_edge_midpoint += pair_axis_translation
        stiffener_snap_residual = shifted_webplate_intersection_target.DistanceTo(
            snapped_pair_shared_edge_midpoint
        )
        stiffener_snap_residual_ok = stiffener_snap_residual <= 1e-6
        bottom_face_unflipped_center = rg.Point3d(snapped_timber_face_dividing_plane.Origin)
        bottom_face_unflipped_center += (
            snapped_timber_face_dividing_plane.ZAxis * (0.5 * thickness)
        )
        bottom_face_flip_about_intersection_edge = mode == "Parallel_to_ground"
        bottom_face_y_axis = rg.Vector3d(snapped_timber_face_dividing_plane.YAxis)
        bottom_face_center_outboard = rg.Point3d(bottom_face_unflipped_center)
        if bottom_face_flip_about_intersection_edge:
            edge_to_unflipped_center = (
                bottom_face_unflipped_center
                - snapped_pair_shared_edge_midpoint
            )
            bottom_face_center_outboard = rg.Point3d(snapped_pair_shared_edge_midpoint)
            bottom_face_center_outboard -= edge_to_unflipped_center
            bottom_face_y_axis.Reverse()
        snapped_bottom_face_center_outboard = rg.Point3d(bottom_face_center_outboard)
        snapped_bottom_face_plane = rg.Plane(
            snapped_bottom_face_center_outboard,
            snapped_timber_face_dividing_plane.XAxis,
            bottom_face_y_axis,
        )
        bottom_face_spec = {
            "center": snapped_bottom_face_center_outboard,
            "plane": snapped_bottom_face_plane,
            "box_dimensions": (timber_width, webplate_depth, thickness),
            "azimuth_deg": plate_spec.get("azimuth_deg"),
            "length": timber_width,
            "width": webplate_depth,
            "height": thickness,
            "thickness": thickness,
            "stiffener_kind": "timber_bottom_face",
            "target_source": "web_plate_targets",
            "target_anchor_member_id": plate_spec.get("member_id"),
            "target_anchor_member_index": plate_spec.get("member_index"),
            "target_anchor_azimuth_deg": plate_spec.get("azimuth_deg"),
            "bottom_face_mode": mode,
            "timber_width": timber_width,
            "timber_height": timber_height,
            "webplate_depth": webplate_depth,
            "webplate_intersection_target": shifted_webplate_intersection_target,
            "webplate_intersection_source": webplate_intersection_source,
            "unsnapped_pair_interface_tip": unsnapped_pair_interface_tip,
            "unsnapped_pair_shared_edge_midpoint": unsnapped_pair_shared_edge_midpoint,
            "snapped_pair_shared_edge_midpoint": snapped_pair_shared_edge_midpoint,
            "stiffener_snap_residual": stiffener_snap_residual,
            "stiffener_snap_residual_ok": stiffener_snap_residual_ok,
            "bottom_face_unflipped_center": bottom_face_unflipped_center,
            "bottom_face_flip_about_intersection_edge": bottom_face_flip_about_intersection_edge,
            "heel_edge_centerline_point": heel_edge_centerline_point,
            "timber_bottom_face_heel_intersection_point": timber_bottom_face_heel_intersection_point,
            "pair_snap_vector": pair_snap_vector,
            "pair_axis_shift": float(pair_axis_shift or 0.0),
            "pair_axis_shift_source": pair_axis_shift_source,
            "pair_axis_shift_vector": pair_axis_translation,
            "heel_side_alignment_y": heel_y,
            "toe_side_alignment_y": toe_y,
            "bottom_face_center_shift": bottom_face_center_shift,
            "collision_centerline_point": collision_centerline_point,
            "collision_point_input": plate_spec.get("collision_point_input"),
            "projected_collision_point": projected_collision_point,
            "edge_projected_collision_point": edge_projected_collision_point,
            "edge_projected_collision_name": edge_projected_collision_name,
            "heel_face_midline_start": heel_face_midline_start,
            "heel_face_midline_end": heel_face_midline_end,
            "heel_face_midpoint_at_timber_face": heel_face_midpoint_at_timber_face,
            "heel_face_intersection_param": heel_face_intersection_param,
            "timber_face_anchor": bottom_face_anchor,
            "timber_face_anchor_source": bottom_face_anchor_source,
            "bottom_face_pivot_point": bottom_face_anchor,
            "bottom_face_pivot_edge_name": pivot_data.get("edge_name"),
            "bottom_face_pivot_edge_start": pivot_data.get("edge_start"),
            "bottom_face_pivot_edge_end": pivot_data.get("edge_end"),
            "bottom_face_pivot_edge_parameter": pivot_data.get("edge_parameter"),
            "bottom_face_pivot_source": bottom_face_anchor_source,
            "timber_face_dividing_plane_origin": snapped_timber_face_dividing_plane.Origin,
            "stiffener_pair_shift_from_point": pair_shift_from_point,
            "stiffener_pair_shift_to_point": pair_shift_to_point,
        }
        specs.append(bottom_face_spec)

        face_intersection_tip = rg.Point3d(snapped_pair_shared_edge_midpoint)
        vertical_height = max(float(face_intersection_tip.Z) - baseplate_top_z, thickness)
        vertical_center = rg.Point3d(
            face_intersection_tip.X,
            face_intersection_tip.Y,
            baseplate_top_z + 0.5 * vertical_height,
        )
        vertical_plane = rg.Plane(vertical_center, width_axis, rg.Vector3d.ZAxis)
        vertical_spec = {
            "center": vertical_center,
            "plane": vertical_plane,
            "box_dimensions": (timber_width, vertical_height, thickness),
            "azimuth_deg": plate_spec.get("azimuth_deg"),
            "length": timber_width,
            "width": vertical_height,
            "height": vertical_height,
            "thickness": thickness,
            "stiffener_kind": "vertical_intersecting",
            "target_source": "web_plate_targets",
            "target_anchor_member_id": plate_spec.get("member_id"),
            "target_anchor_member_index": plate_spec.get("member_index"),
            "target_anchor_azimuth_deg": plate_spec.get("azimuth_deg"),
            "bottom_face_mode": mode,
            "timber_width": timber_width,
            "timber_height": timber_height,
            "webplate_depth": webplate_depth,
            "webplate_intersection_target": shifted_webplate_intersection_target,
            "webplate_intersection_source": webplate_intersection_source,
            "unsnapped_pair_interface_tip": unsnapped_pair_interface_tip,
            "unsnapped_pair_shared_edge_midpoint": unsnapped_pair_shared_edge_midpoint,
            "snapped_pair_shared_edge_midpoint": snapped_pair_shared_edge_midpoint,
            "stiffener_snap_residual": stiffener_snap_residual,
            "stiffener_snap_residual_ok": stiffener_snap_residual_ok,
            "bottom_face_unflipped_center": bottom_face_unflipped_center,
            "bottom_face_flip_about_intersection_edge": bottom_face_flip_about_intersection_edge,
            "heel_edge_centerline_point": heel_edge_centerline_point,
            "timber_bottom_face_heel_intersection_point": timber_bottom_face_heel_intersection_point,
            "pair_snap_vector": pair_snap_vector,
            "pair_axis_shift": float(pair_axis_shift or 0.0),
            "pair_axis_shift_source": pair_axis_shift_source,
            "pair_axis_shift_vector": pair_axis_translation,
            "face_intersection_tip": face_intersection_tip,
            "face_intersection_tip_y": heel_y,
            "collision_centerline_point": collision_centerline_point,
            "collision_point_input": plate_spec.get("collision_point_input"),
            "projected_collision_point": projected_collision_point,
            "edge_projected_collision_point": edge_projected_collision_point,
            "edge_projected_collision_name": edge_projected_collision_name,
            "heel_face_midline_start": heel_face_midline_start,
            "heel_face_midline_end": heel_face_midline_end,
            "heel_face_midpoint_at_timber_face": heel_face_midpoint_at_timber_face,
            "heel_face_intersection_param": heel_face_intersection_param,
            "timber_face_anchor": bottom_face_anchor,
            "timber_face_anchor_source": bottom_face_anchor_source,
            "bottom_face_pivot_point": bottom_face_anchor,
            "bottom_face_pivot_edge_name": pivot_data.get("edge_name"),
            "bottom_face_pivot_edge_start": pivot_data.get("edge_start"),
            "bottom_face_pivot_edge_end": pivot_data.get("edge_end"),
            "bottom_face_pivot_edge_parameter": pivot_data.get("edge_parameter"),
            "bottom_face_pivot_source": bottom_face_anchor_source,
            "timber_face_dividing_plane_origin": snapped_timber_face_dividing_plane.Origin,
            "stiffener_pair_shift_from_point": pair_shift_from_point,
            "stiffener_pair_shift_to_point": pair_shift_to_point,
        }
        specs.append(vertical_spec)
    return specs


def _default_plate_hole_specs(plane):
    specs = []
    for spec in PLATE_HOLE_REFERENCE:
        local = (
            spec["center"][0] - SOURCE_FULL_PLATE_CENTER[0],
            spec["center"][1] - SOURCE_FULL_PLATE_CENTER[1],
            spec["center"][2] - SOURCE_FULL_PLATE_CENTER[2],
        )
        next_spec = dict(spec)
        next_spec["center"] = _point_on_plane(plane, local)
        next_spec.update(CODE_BASELINE_PLATE_HOLES)
        next_spec["dimension_source"] = "code_baseline"
        specs.append(next_spec)
    return specs


def _point_local_in_plane(point, plane):
    if rg is None or point is None or plane is None:
        return (0.0, 0.0, 0.0)
    vector = point - plane.Origin
    return (
        vector.X * plane.XAxis.X + vector.Y * plane.XAxis.Y + vector.Z * plane.XAxis.Z,
        vector.X * plane.YAxis.X + vector.Y * plane.YAxis.Y + vector.Z * plane.YAxis.Z,
        vector.X * plane.ZAxis.X + vector.Y * plane.ZAxis.Y + vector.Z * plane.ZAxis.Z,
    )


def _nearest_plate_edge_point(plate_spec, plate_plane, point):
    """Return the nearest point on a rectangular webplate outline edge."""
    if rg is None or plate_plane is None or point is None:
        return None, None

    local_x, local_y, _ = _point_local_in_plane(point, plate_plane)
    half_length = 0.5 * max(float(plate_spec.get("length") or 0.0), 0.0)
    half_width = 0.5 * max(float(plate_spec.get("width") or 0.0), 0.0)
    if half_length <= 1e-9 or half_width <= 1e-9:
        return None, None

    edge_segments = {
        "heel": ((-half_length, -half_width), (-half_length, half_width)),
        "tip": ((half_length, -half_width), (half_length, half_width)),
        "depth_neg": ((-half_length, -half_width), (half_length, -half_width)),
        "depth_pos": ((-half_length, half_width), (half_length, half_width)),
    }
    candidates = []
    for edge_name, (start, end) in edge_segments.items():
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            continue
        t = ((local_x - start[0]) * dx + (local_y - start[1]) * dy) / length_sq
        t = max(0.0, min(1.0, t))
        edge_x = start[0] + t * dx
        edge_y = start[1] + t * dy
        distance_sq = (local_x - edge_x) ** 2 + (local_y - edge_y) ** 2
        candidates.append((distance_sq, edge_x, edge_y, edge_name))
    if not candidates:
        return None, None
    _, edge_x, edge_y, edge_name = min(candidates, key=lambda item: item[0])

    return _point_from_plane_local(plate_plane, (edge_x, edge_y, 0.0)), edge_name


def _plate_edge_segment(plate_spec, plate_plane, edge_name):
    """Return the selected rectangular webplate edge as a world-space segment."""
    if rg is None or plate_plane is None:
        return None, None
    half_length = 0.5 * max(float(plate_spec.get("length") or 0.0), 0.0)
    half_width = 0.5 * max(float(plate_spec.get("width") or 0.0), 0.0)
    if half_length <= 1e-9 or half_width <= 1e-9:
        return None, None
    edge_locals = {
        "heel": ((-half_length, -half_width), (-half_length, half_width)),
        "tip": ((half_length, -half_width), (half_length, half_width)),
        "depth_neg": ((-half_length, -half_width), (half_length, -half_width)),
        "depth_pos": ((-half_length, half_width), (half_length, half_width)),
    }
    local_segment = edge_locals.get(edge_name)
    if local_segment is None:
        return None, None
    start_local, end_local = local_segment
    return (
        _point_from_plane_local(plate_plane, (start_local[0], start_local[1], 0.0)),
        _point_from_plane_local(plate_plane, (end_local[0], end_local[1], 0.0)),
    )


def _plate_selected_face_midline_segment(plate_spec, plate_plane, edge_name):
    """Return the mid-thickness rail on the selected side face of a webplate."""
    if edge_name not in ("depth_neg", "depth_pos"):
        return None, None
    return _plate_edge_segment(plate_spec, plate_plane, edge_name)


def _point_on_line_at_world_z(line_start, line_end, target_z):
    if rg is None or line_start is None or line_end is None or target_z is None:
        return None, None
    dz = float(line_end.Z) - float(line_start.Z)
    if abs(dz) <= 1e-9:
        return None, None
    parameter = (float(target_z) - float(line_start.Z)) / dz
    direction = line_end - line_start
    point = rg.Point3d(line_start)
    point += direction * parameter
    return point, parameter


def _bottom_face_pivot_data(
    plate_spec,
    plate_plane,
    collision_centerline_point,
    effective_timber_bottom_z,
):
    """Resolve the width-edge hinge used by both timber bottom-face modes."""
    edge_name = plate_spec.get("edge_projected_collision_name")
    if edge_name not in ("depth_neg", "depth_pos"):
        projected = plate_spec.get("projected_collision_point")
        if projected is not None:
            projected_local = _point_local_in_plane(projected, plate_plane)
            edge_name = "depth_neg" if float(projected_local[1]) <= 0.0 else "depth_pos"
        else:
            edge_name = "depth_neg"

    edge_start, edge_end = _plate_selected_face_midline_segment(
        plate_spec,
        plate_plane,
        edge_name,
    )
    pivot_point, pivot_param = _point_on_line_at_world_z(
        edge_start,
        edge_end,
        effective_timber_bottom_z,
    )
    pivot_source = "selected_depth_edge_at_effective_z"
    if pivot_point is None:
        pivot_point = plate_spec.get("edge_projected_collision_point")
        pivot_source = "edge_projection_fallback"
    if pivot_point is None:
        pivot_point = collision_centerline_point
        pivot_source = "collision_centerline_fallback"
    return {
        "point": rg.Point3d(pivot_point) if rg is not None and pivot_point is not None else pivot_point,
        "edge_name": edge_name,
        "edge_start": edge_start,
        "edge_end": edge_end,
        "edge_parameter": pivot_param,
        "source": pivot_source,
    }


def _line_plane_intersection_point(line_start, line_end, plane):
    """Intersect an infinite line through two points with a plane."""
    if rg is None or line_start is None or line_end is None or plane is None:
        return None, None
    line_direction = line_end - line_start
    denominator = rg.Vector3d.Multiply(plane.ZAxis, line_direction)
    if abs(float(denominator)) <= 1e-9:
        return None, None
    numerator = rg.Vector3d.Multiply(plane.ZAxis, plane.Origin - line_start)
    parameter = float(numerator) / float(denominator)
    point = rg.Point3d(line_start)
    point += line_direction * parameter
    return point, parameter


def _point_from_plane_local(plane, local_xyz):
    if rg is None or plane is None:
        return local_xyz
    point = rg.Point3d(plane.Origin)
    point += plane.XAxis * float(local_xyz[0])
    point += plane.YAxis * float(local_xyz[1])
    point += plane.ZAxis * float(local_xyz[2])
    return point


def _translated_plane_world_z(plane, delta_z):
    if rg is None or plane is None:
        return plane
    origin = rg.Point3d(plane.Origin)
    origin.Z += float(delta_z)
    return rg.Plane(origin, plane.XAxis, plane.YAxis)


def _reoriented_cluster_point(
    point,
    source_cluster_origin,
    source_azimuth_deg,
    source_altitude_deg,
    target_cluster_plane,
    target_azimuth_deg,
    target_altitude_deg,
):
    if rg is None or target_cluster_plane is None or point is None:
        return point

    source_plane = _plate_plane(
        source_cluster_origin,
        source_azimuth_deg,
        source_altitude_deg,
    )
    target_plane = _plate_plane(
        target_cluster_plane.Origin,
        target_azimuth_deg,
        target_altitude_deg,
    )
    if source_plane is None or target_plane is None:
        return point

    local = _point_local_in_plane(point, source_plane)
    return _point_from_plane_local(target_plane, local)


def _member_axis_aligned_cluster_point(
    point,
    source_cluster_origin,
    source_azimuth_deg,
    source_altitude_deg,
    target_cluster_plane,
    target_azimuth_deg,
    target_altitude_deg,
):
    """Project a legacy plate center onto the live member axis.

    The old Rhino footing references encode substantial normal offsets from the
    support node. Those offsets are useful for reproducing the source assembly,
    but they must not be carried into line-model-driven plate placement.
    """
    if rg is None or target_cluster_plane is None or point is None:
        return point

    source_plane = _plate_plane(
        source_cluster_origin,
        source_azimuth_deg,
        source_altitude_deg,
    )
    target_plane = _plate_plane(
        target_cluster_plane.Origin,
        target_azimuth_deg,
        target_altitude_deg,
    )
    if source_plane is None or target_plane is None:
        return point

    local_x, _local_y, _local_z = _point_local_in_plane(point, source_plane)
    return _point_from_plane_local(target_plane, (local_x, 0.0, 0.0))


def _plate_support_end_points(spec):
    if rg is None or not spec:
        return []
    plane = _plate_plane(
        spec["center"],
        spec["azimuth_deg"],
        spec["inclination_deg"],
    )
    if plane is None:
        return []

    half_length = 0.5 * float(spec.get("length") or 0.0)
    half_width = 0.5 * float(spec.get("width") or 0.0)
    points = []
    for y in (-half_width, half_width):
        point = rg.Point3d(plane.Origin)
        point += plane.XAxis * (-half_length)
        point += plane.YAxis * y
        points.append(point)
    return points


def _plate_support_heel_point(spec):
    if rg is None or not spec:
        return None
    plane = _plate_plane(
        spec["center"],
        spec["azimuth_deg"],
        spec["inclination_deg"],
    )
    if plane is None:
        return None

    local = spec.get("support_heel_local")
    if local is not None:
        reference_length = float(spec.get("reference_length") or spec.get("length") or 1.0)
        reference_width = float(spec.get("reference_width") or spec.get("width") or 1.0)
        reference_thickness = float(spec.get("reference_thickness") or spec.get("thickness") or 1.0)
        scaled_local = (
            float(local[0]) * float(spec.get("length") or reference_length) / reference_length,
            float(local[1]) * float(spec.get("width") or reference_width) / reference_width,
            float(local[2]) * float(spec.get("thickness") or reference_thickness) / reference_thickness,
        )
        return _point_from_plane_local(plane, scaled_local)

    support_points = _plate_support_end_points(spec)
    return max(support_points, key=lambda point: point.Z) if support_points else None


def _plate_support_heel_points(specs):
    heels = []
    for spec in specs or []:
        heel = _plate_support_heel_point(spec)
        if heel is not None:
            heels.append(heel)
    return heels


def _plate_support_heel_z_values(specs):
    return [point.Z for point in _plate_support_heel_points(specs)]


def _canonicalize_straight_plate_heel_specs(specs):
    """Make straight active plate bodies start at their true heel datum.

    The source Rhino plates carry a heel local point from the curved/filleted
    transition geometry. Once we build a separate straight rectangular body,
    that legacy interior x-offset no longer describes the body start. Keeping
    it makes collision-driven lengths and hole offsets reference a different
    origin from the visible straight plate end. For a sloped rectangular plate,
    the heel is the higher base corner; the lower base corner is the toe.
    """
    canonical = []
    for spec in specs or []:
        next_spec = dict(spec)
        source_local = next_spec.get("support_heel_local")
        reference_length = float(next_spec.get("reference_length") or next_spec.get("length") or 0.0)
        reference_width = float(next_spec.get("reference_width") or next_spec.get("width") or 0.0)
        if reference_length > 0.0 and reference_width > 0.0:
            plane = _plate_plane(
                next_spec["center"],
                next_spec["azimuth_deg"],
                next_spec["inclination_deg"],
            )
            # Local +Y slopes downward for the current web-plate frame, so
            # choose the base-corner side whose Y offset gives the higher Z.
            if plane is not None:
                heel_y_sign = 1.0 if plane.YAxis.Z >= 0.0 else -1.0
            else:
                heel_y_sign = -1.0
            next_spec["source_support_heel_local"] = source_local
            next_spec["support_heel_local"] = (
                -0.5 * reference_length,
                heel_y_sign * 0.5 * reference_width,
                0.0,
            )
            next_spec["support_toe_local"] = (
                -0.5 * reference_length,
                -heel_y_sign * 0.5 * reference_width,
                0.0,
            )
            next_spec["heel_datum_source"] = "higher_straight_body_base_corner"
        else:
            next_spec["heel_datum_source"] = "source_reference"
        canonical.append(next_spec)
    return canonical


def _shift_plate_specs_to_heel_z(specs, target_z):
    if rg is None or target_z is None:
        return list(specs or [])

    shifted = []
    for spec in specs or []:
        next_spec = dict(spec)
        heel = _plate_support_heel_point(next_spec)
        plane = _plate_plane(
            next_spec["center"],
            next_spec["azimuth_deg"],
            next_spec["inclination_deg"],
        )
        axis_z = plane.XAxis.Z if plane is not None else 0.0
        if heel is not None and abs(axis_z) > 1e-9:
            axis_shift = (float(target_z) - heel.Z) / axis_z
            center = rg.Point3d(next_spec["center"])
            center += plane.XAxis * axis_shift
            next_spec["center"] = center
            next_spec["heel_axis_shift"] = axis_shift
            next_spec["heel_target_z"] = float(target_z)
        else:
            next_spec["heel_axis_shift"] = None
            next_spec["heel_target_z"] = float(target_z)
        shifted.append(next_spec)
    return shifted


def _median(values, default=0.0):
    items = sorted(values)
    if not items:
        return default
    mid = len(items) // 2
    if len(items) % 2:
        return items[mid]
    return 0.5 * (items[mid - 1] + items[mid])


def _cluster_values(values, tolerance=1.0):
    groups = []
    for value in sorted(values):
        if not groups or abs(value - _median(groups[-1])) > tolerance:
            groups.append([value])
        else:
            groups[-1].append(value)
    return groups


def _hole_face_circle(face):
    try:
        if face.IsPlanar():
            return None
        domain = face.Domain(0)
        curve = face.IsoCurve(1, 0.5 * (domain.T0 + domain.T1))
        if curve is None or not curve.IsCircle():
            return None
        result = curve.TryGetCircle()
        if isinstance(result, tuple):
            ok, circle = result
            return circle if ok else None
        return result
    except Exception:
        return None


def extract_plate_hole_specs(reference_full_plate_breps, full_plate_specs=None):
    refs = _flatten_values(reference_full_plate_breps)
    if not refs:
        return []

    if not full_plate_specs:
        full_plate_specs = extract_oriented_brep_specs(
            refs,
            target_width=140.0,
            target_thickness=10.0,
        )

    specs = []
    for index, brep in enumerate(refs):
        if brep is None or not hasattr(brep, "Faces"):
            continue
        plate_spec = full_plate_specs[index] if index < len(full_plate_specs) else None
        plane = plate_spec.get("plane") if plate_spec else None
        if plane is None:
            continue

        circles = []
        for face in brep.Faces:
            circle = _hole_face_circle(face)
            if circle is not None:
                circles.append(circle)
        if not circles:
            continue

        centers = [circle.Center for circle in circles]
        diameter = _median([2.0 * circle.Radius for circle in circles])
        center = rg.Point3d(
            sum(point.X for point in centers) / len(centers),
            sum(point.Y for point in centers) / len(centers),
            sum(point.Z for point in centers) / len(centers),
        )

        local = []
        for point in centers:
            vector = point - center
            local.append(
                (
                    vector.X * plane.XAxis.X + vector.Y * plane.XAxis.Y + vector.Z * plane.XAxis.Z,
                    vector.X * plane.YAxis.X + vector.Y * plane.YAxis.Y + vector.Z * plane.YAxis.Z,
                )
            )

        row_groups = _cluster_values([item[1] for item in local], tolerance=1.0)
        row_count = 2 if len(row_groups) >= 2 else 1
        row_centers = [_median(group) for group in row_groups]
        row_spacing = abs(row_centers[-1] - row_centers[0]) if len(row_centers) >= 2 else 0.0
        long_groups = _cluster_values([item[0] for item in local], tolerance=1.0)
        long_centers = [_median(group) for group in long_groups]
        pitches = [
            long_centers[item + 1] - long_centers[item]
            for item in range(len(long_centers) - 1)
        ]
        pitch = _median(pitches, 0.0)
        holes_per_row = max(1, int(round(float(len(circles)) / float(row_count))))

        specs.append(
            {
                "source_index": index,
                "center": center,
                "row_count": row_count,
                "holes_per_row": holes_per_row,
                "diameter": diameter,
                "row_spacing": row_spacing,
                "pitch": pitch,
                "hole_centers": centers,
            }
        )
    return specs


def _combined_stiffener_specs_from_source(plane, source_brep):
    if rg is None or source_brep is None or not hasattr(source_brep, "Faces"):
        return []

    specs = []
    for reference in STIFFENER_REFERENCE:
        face_indices = reference.get("source_faces") or ()
        if len(face_indices) != 2:
            continue

        faces = []
        for face_index in face_indices:
            if face_index < 0 or face_index >= len(source_brep.Faces):
                faces = []
                break
            faces.append(source_brep.Faces[face_index])
        if len(faces) != 2:
            continue

        points = []
        for face in faces:
            duplicate = face.DuplicateFace(False)
            for vertex in duplicate.Vertices:
                point = vertex.Location
                if not any(point.DistanceTo(existing) <= 0.01 for existing in points):
                    points.append(point)
        if len(points) < 4:
            continue

        center = rg.Point3d(
            sum(point.X for point in points) / len(points),
            sum(point.Y for point in points) / len(points),
            sum(point.Z for point in points) / len(points),
        )

        bb0 = faces[0].GetBoundingBox(True)
        bb1 = faces[1].GetBoundingBox(True)
        c0 = bb0.Center
        c1 = bb1.Center
        thickness_axis = c1 - c0
        if not thickness_axis.Unitize():
            continue

        x_axis = rg.Vector3d.CrossProduct(rg.Vector3d.ZAxis, thickness_axis)
        if not x_axis.Unitize():
            continue

        local = []
        for point in points:
            vector = point - center
            local.append(
                (
                    vector.X * x_axis.X + vector.Y * x_axis.Y + vector.Z * x_axis.Z,
                    vector.Z,
                    (
                        vector.X * thickness_axis.X
                        + vector.Y * thickness_axis.Y
                        + vector.Z * thickness_axis.Z
                    ),
                )
            )

        min_x = min(item[0] for item in local)
        max_x = max(item[0] for item in local)
        min_z = min(item[1] for item in local)
        max_z = max(item[1] for item in local)
        min_t = min(item[2] for item in local)
        max_t = max(item[2] for item in local)
        azimuth = math.degrees(math.atan2(x_axis.Y, x_axis.X))

        next_spec = dict(reference)
        next_spec["center"] = center
        next_spec["length"] = max_x - min_x
        next_spec["height"] = max_z - min_z
        next_spec["thickness"] = min(reference.get("thickness", 10.0), max_t - min_t)
        next_spec["azimuth_deg"] = azimuth
        specs.append(next_spec)
    return specs


def _plate_specs_from_inputs(
    plane,
    reference_plates=None,
    timber_elements=None,
    plate_timber_indices=None,
    plate_member_indices=None,
    plate_member_ids=None,
    plate_orientation_sources=None,
    plate_centers=None,
    plate_azimuths=None,
    plate_altitudes=None,
    plate_inclinations=None,
    plate_lengths=None,
    plate_widths=None,
    plate_thicknesses=None,
):
    if rg is not None and reference_plates:
        specs = extract_plate_specs(reference_plates)
    else:
        specs = _default_plate_specs(plane)
    if not specs:
        specs = _default_plate_specs(plane)

    input_centers = _flatten_values(plate_centers)
    count = max(
        len(specs),
        len(input_centers),
        len(_flatten_values(plate_azimuths)),
        len(_flatten_values(plate_altitudes)),
        len(_flatten_values(plate_inclinations)),
        len(_flatten_values(plate_lengths)),
        len(_flatten_values(plate_widths)),
        len(_flatten_values(plate_thicknesses)),
    )
    if count <= 0:
        return []

    result = []
    for index in range(count):
        base = dict(specs[index] if index < len(specs) else specs[-1])
        base["reference_length"] = base.get("length") or 1.0
        base["reference_height"] = base.get("height", base.get("width")) or 1.0
        base["reference_width"] = base.get("width") or 1.0
        base["reference_thickness"] = base.get("thickness") or 1.0
        source_center = _point_from_value(
            base.get("source_center"),
            base.get("center"),
        )
        source_azimuth = base.get("source_azimuth_deg", base.get("azimuth_deg"))
        source_altitude = base.get("altitude_deg", base.get("inclination_deg"))
        base["source_azimuth_deg"] = source_azimuth
        target_azimuth = _float_at(plate_azimuths, index, base.get("azimuth_deg"))
        timber_altitude = _timber_altitude_at(timber_elements, plate_timber_indices, index)
        altitude = timber_altitude if timber_altitude is not None else source_altitude
        explicit_altitude = _float_at(plate_altitudes, index, None)
        legacy_inclination = _float_at(plate_inclinations, index, None)
        explicit_orientation_source = _list_at(
            _flatten_values(plate_orientation_sources),
            index,
            None,
        )
        if explicit_altitude is not None:
            altitude = explicit_altitude
        elif legacy_inclination is not None:
            altitude = legacy_inclination
        explicit_center = _point_from_value(_list_at(input_centers, index), None)
        if explicit_center is not None:
            center = explicit_center
        elif (
            rg is not None
            and source_center is not None
            and source_azimuth is not None
            and source_altitude is not None
        ):
            if str(explicit_orientation_source or "") == "line_model_member":
                center = _member_axis_aligned_cluster_point(
                    source_center,
                    rg.Point3d(*SOURCE_WEB_PLATE_CENTER),
                    source_azimuth,
                    source_altitude,
                    plane,
                    target_azimuth,
                    altitude,
                )
                base["center_alignment_source"] = "line_model_member_axis"
            else:
                center = _reoriented_cluster_point(
                    source_center,
                    rg.Point3d(*SOURCE_WEB_PLATE_CENTER),
                    source_azimuth,
                    source_altitude,
                    plane,
                    target_azimuth,
                    altitude,
                )
                base["center_alignment_source"] = "rhino_geometry"
        else:
            center = base.get("center")
            base["center_alignment_source"] = "fallback"
        base["center"] = center
        base["azimuth_deg"] = target_azimuth
        base["altitude_deg"] = altitude
        base["inclination_deg"] = altitude

        base["length"] = _float_at(plate_lengths, index, base.get("length"))
        base["width"] = _float_at(plate_widths, index, base.get("width"))
        base["thickness"] = _float_at(plate_thicknesses, index, base.get("thickness"))
        base["timber_index"] = _int_at(plate_timber_indices, index, index if timber_altitude is not None else None)
        base["member_index"] = _int_at(plate_member_indices, index, None)
        base["member_id"] = _list_at(_flatten_values(plate_member_ids), index, None)
        altitude_source = (
            "override"
            if explicit_altitude is not None or legacy_inclination is not None
            else "timber_element"
            if timber_altitude is not None
            else "rhino_geometry"
        )
        if explicit_orientation_source is not None:
            altitude_source = str(explicit_orientation_source)
        base["altitude_source"] = altitude_source
        base["orientation_source"] = altitude_source
        result.append(base)
    return result


def _full_plate_specs_from_inputs(
    plane,
    ref_full_plate_breps,
    plate_specs,
    plate_full_centers=None,
    plate_full_lengths=None,
):
    if not plate_specs:
        plate_specs = _default_plate_specs(plane)
    if rg is not None and ref_full_plate_breps:
        specs = extract_oriented_brep_specs(ref_full_plate_breps, target_width=140.0, target_thickness=10.0)
    else:
        specs = _default_full_plate_specs(plane)
    if not specs:
        specs = _default_full_plate_specs(plane)

    count = max(len(specs), len(plate_specs), len(_flatten_values(plate_full_lengths)))
    input_centers = _flatten_values(plate_full_centers)
    count = max(count, len(input_centers))
    matched_sources = _matched_specs_by_azimuth(specs, plate_specs)
    result = []
    for index in range(count):
        target_plate = plate_specs[index] if index < len(plate_specs) else plate_specs[-1]
        if index < len(matched_sources):
            source_spec_index, source_spec = matched_sources[index]
        else:
            source_spec_index = min(index, len(specs) - 1)
            source_spec = specs[source_spec_index]
        base = dict(source_spec)
        base["source_spec_index"] = source_spec_index
        base["source_reference_azimuth_deg"] = source_spec.get("azimuth_deg")
        source_center = base.get("center")
        source_azimuth = base.get("azimuth_deg")
        source_altitude = base.get("altitude_deg", base.get("inclination_deg"))
        target_azimuth = target_plate.get("azimuth_deg", source_azimuth)
        target_altitude = target_plate.get(
            "altitude_deg",
            target_plate.get("inclination_deg", source_altitude),
        )
        explicit_center = _point_from_value(_list_at(input_centers, index), None)
        if explicit_center is not None:
            base["center"] = explicit_center
        elif (
            rg is not None
            and source_center is not None
            and source_azimuth is not None
            and source_altitude is not None
        ):
            base["center"] = _reoriented_cluster_point(
                source_center,
                rg.Point3d(*SOURCE_FULL_PLATE_CENTER),
                source_azimuth,
                source_altitude,
                plane,
                target_azimuth,
                target_altitude,
            )
        else:
            base["center"] = source_center
        base["azimuth_deg"] = target_azimuth
        base["altitude_deg"] = target_altitude
        base["inclination_deg"] = target_altitude
        base["straight_length"] = target_plate.get("length", base.get("straight_length", base.get("length")))
        base["width"] = target_plate.get("width", base.get("width"))
        base["thickness"] = target_plate.get("thickness", base.get("thickness"))
        default_full_length = base.get("full_length", base.get("length"))
        base["full_length"] = _float_at(plate_full_lengths, index, default_full_length)
        base["length"] = base["full_length"]
        base["plane"] = _plate_plane(
            base["center"],
            base["azimuth_deg"],
            base["inclination_deg"],
        )
        result.append(base)
    return result


def _full_plate_transform_scales(source, target, index):
    fallback = FULL_PLATE_REFERENCE[min(index, len(FULL_PLATE_REFERENCE) - 1)]
    source_straight = source.get("straight_length") or fallback["straight_length"]
    scale_x = (target.get("straight_length") or target.get("length") or source_straight) / source_straight
    scale_y = (target.get("width") or source.get("width") or 1.0) / (source.get("width") or 1.0)
    scale_z = (target.get("thickness") or source.get("thickness") or 1.0) / (source.get("thickness") or 1.0)
    return scale_x, scale_y, scale_z


def _transform_plate_point(point, source_spec, target_spec, index):
    if rg is None:
        return point
    source_plane = source_spec.get("plane")
    target_plane = _plate_plane(
        target_spec["center"],
        target_spec["azimuth_deg"],
        target_spec["inclination_deg"],
    )
    if source_plane is None or target_plane is None:
        return point
    local_x, local_y, local_z = _point_local_in_plane(point, source_plane)
    scale_x, scale_y, scale_z = _full_plate_transform_scales(source_spec, target_spec, index)
    return _point_from_plane_local(
        target_plane,
        (local_x * scale_x, local_y * scale_y, local_z * scale_z),
    )


def _transformed_source_plate_hole_specs(source_hole_specs, source_full_plate_specs, target_full_plate_specs):
    result = []
    for index, target_spec in enumerate(target_full_plate_specs or []):
        source_index = int(target_spec.get("source_spec_index", index))
        if source_index >= len(source_hole_specs) or source_index >= len(source_full_plate_specs):
            continue
        source_hole = dict(source_hole_specs[source_index])
        source_spec = source_full_plate_specs[source_index]
        source_hole["center"] = _transform_plate_point(
            source_hole["center"],
            source_spec,
            target_spec,
            index,
        )
        result.append(source_hole)
    return result


def _active_plate_hole_specs_for_web_plates(
    hole_specs,
    plate_specs,
    preserve_input_centers=False,
):
    """Return hole specs centered on the active straight web plates.

    The curved/full-plate references live in a different source neighborhood
    from the straight web plates. Their hole dimensions are still useful, but
    their world-space centers are not valid cutters for the straight Breps.
    """
    if not hole_specs or not plate_specs:
        return []

    result = []
    for index, plate_spec in enumerate(plate_specs):
        source = hole_specs[index] if index < len(hole_specs) else hole_specs[-1]
        next_spec = dict(source)
        if preserve_input_centers and next_spec.get("center") is not None:
            next_spec["active_center_source"] = "override"
        elif (
            plate_spec.get("collision_axis_distance") is not None
            and plate_spec.get("hole_end_distance") is not None
        ):
            heel = _plate_support_heel_point(plate_spec)
            plate_plane = _plate_plane(
                plate_spec["center"],
                plate_spec["azimuth_deg"],
                plate_spec["inclination_deg"],
            )
            holes_per_row = max(1, int(next_spec.get("holes_per_row", 1)))
            pitch = float(next_spec.get("pitch") or 0.0)
            stagger_offset = (
                float(next_spec.get("stagger_offset") or 0.0)
                if next_spec.get("row_mode") == "staggered_double_row"
                else 0.0
            )
            pattern_span = max(0, holes_per_row - 1) * pitch + stagger_offset
            if heel is not None and plate_plane is not None:
                center_offset = (
                    float(plate_spec["collision_axis_distance"])
                    + float(
                        plate_spec.get("bottom_hole_end_distance")
                        or plate_spec["hole_end_distance"]
                    )
                    + 0.5 * pattern_span
                )
                heel_local = _point_local_in_plane(heel, plate_plane)
                # Longitudinal distances are measured from the heel, but the
                # paired hole rows must remain centered across the plate width.
                center = _point_on_plane(
                    plate_plane,
                    (heel_local[0] + center_offset, 0.0, 0.0),
                )
                next_spec["center"] = center
                next_spec["active_center_source"] = "collision_reference_centerline"
            else:
                next_spec["center"] = plate_spec.get("center")
                next_spec["active_center_source"] = "plate_center_fallback"
        else:
            next_spec["center"] = plate_spec.get("center")
            next_spec["active_center_source"] = "plate_center"
        result.append(next_spec)
    return result


def _full_plate_transform_changed(source_full_plate_specs, target_full_plate_specs, tolerance=1e-6):
    for index, target_spec in enumerate(target_full_plate_specs or []):
        source_index = int(target_spec.get("source_spec_index", index))
        if source_index >= len(source_full_plate_specs or []):
            continue
        scale_x, scale_y, scale_z = _full_plate_transform_scales(
            source_full_plate_specs[source_index],
            target_spec,
            index,
        )
        if (
            abs(scale_x - 1.0) > tolerance
            or abs(scale_y - 1.0) > tolerance
            or abs(scale_z - 1.0) > tolerance
        ):
            return True
    return False


def _plate_hole_pattern_changed(source_hole_specs, target_hole_specs, tolerance=1e-9):
    count = min(len(source_hole_specs or []), len(target_hole_specs or []))
    if count == 0:
        return bool(source_hole_specs or target_hole_specs)
    keys = ("row_count", "holes_per_row", "diameter", "row_spacing", "pitch", "stagger_offset", "row_mode")
    for index in range(count):
        source = source_hole_specs[index]
        target = target_hole_specs[index]
        for key in keys:
            source_value = source.get(key)
            target_value = target.get(key)
            if key in ("row_count", "holes_per_row"):
                if int(source_value or 0) != int(target_value or 0):
                    return True
            elif key == "row_mode":
                if str(source_value or "") != str(target_value or ""):
                    return True
            elif abs(float(source_value or 0.0) - float(target_value or 0.0)) > tolerance:
                return True
    return len(source_hole_specs or []) != len(target_hole_specs or [])


def _plate_hole_specs_from_inputs(
    plane,
    source_hole_specs,
    source_full_plate_specs,
    target_full_plate_specs,
    plate_hole_centers=None,
    plate_hole_rows=None,
    plate_hole_patterns=None,
    plate_holes_per_row=None,
    plate_hole_diameters=None,
    plate_bolt_diameters=None,
    plate_hole_clearances=None,
    plate_total_hole_counts=None,
    plate_hole_row_spacings=None,
    plate_hole_pitches=None,
    plate_hole_stagger_offsets=None,
    collapse_single_row_from_pairs=True,
):
    specs = list(source_hole_specs or _default_plate_hole_specs(plane))
    if not specs:
        specs = _default_plate_hole_specs(plane)

    transformed_defaults = _transformed_source_plate_hole_specs(
        specs,
        source_full_plate_specs,
        target_full_plate_specs,
    )
    if transformed_defaults:
        specs = transformed_defaults
    specs = [
        dict(spec, **CODE_BASELINE_PLATE_HOLES, dimension_source="code_baseline")
        for spec in specs
    ]

    input_centers = _flatten_values(plate_hole_centers)
    count = max(
        len(specs),
        len(input_centers),
        len(_flatten_values(plate_hole_rows)),
        len(_flatten_values(plate_hole_patterns)),
        len(_flatten_values(plate_holes_per_row)),
        len(_flatten_values(plate_hole_diameters)),
        len(_flatten_values(plate_bolt_diameters)),
        len(_flatten_values(plate_hole_clearances)),
        len(_flatten_values(plate_total_hole_counts)),
        len(_flatten_values(plate_hole_row_spacings)),
        len(_flatten_values(plate_hole_pitches)),
        len(_flatten_values(plate_hole_stagger_offsets)),
    )
    if count <= 0:
        return []

    result = []
    for index in range(count):
        base = dict(specs[index] if index < len(specs) else specs[-1])
        source_row_count = _row_count_at(
            None,
            index,
            base.get("row_count", CODE_BASELINE_PLATE_HOLES["row_count"]),
        )
        source_holes_per_row = max(
            1,
            int(base.get("holes_per_row", CODE_BASELINE_PLATE_HOLES["holes_per_row"]) or 1),
        )
        requested_row_count = _row_count_at(
            plate_hole_rows,
            index,
            base.get("row_count", CODE_BASELINE_PLATE_HOLES["row_count"]),
        )
        requested_mode_value = _int_at(plate_hole_rows, index, None)
        requested_pattern = _plate_hole_pattern_at(
            plate_hole_patterns,
            index,
            base.get("row_mode")
            or ("single_row_centerline" if requested_row_count == 1 else "double_row"),
        )
        if requested_mode_value == 3 and not _has_values(plate_hole_patterns):
            requested_pattern = "staggered_double_row"
        elif requested_mode_value == 2 and not _has_values(plate_hole_patterns):
            requested_pattern = "double_row"
        elif requested_mode_value == 1 and not _has_values(plate_hole_patterns):
            requested_pattern = "single_row_centerline"
        if requested_pattern == "single_row_centerline":
            requested_row_count = 1
        elif requested_pattern in ("double_row", "staggered_double_row"):
            requested_row_count = 2
        base["center"] = _point_from_value(_list_at(input_centers, index), base.get("center"))
        base["row_count"] = requested_row_count
        requested_holes_per_row = max(
            1,
            _int_at(
                plate_holes_per_row,
                index,
                source_holes_per_row,
            ),
        )
        requested_total_bolt_count = _int_at(plate_total_hole_counts, index, None)
        if requested_total_bolt_count is not None:
            requested_total_bolt_count = max(1, int(requested_total_bolt_count))
            requested_holes_per_row = max(
                1,
                int(math.ceil(float(requested_total_bolt_count) / float(max(requested_row_count, 1)))),
            )
        base["source_row_count"] = source_row_count
        base["source_holes_per_row"] = source_holes_per_row
        base["source_total_hole_count"] = source_row_count * source_holes_per_row
        base["holes_per_row_input"] = requested_holes_per_row
        if requested_row_count == 1:
            explicit_holes_input = _has_values(plate_holes_per_row)
            if requested_total_bolt_count is not None:
                total_holes = requested_total_bolt_count
            elif collapse_single_row_from_pairs:
                pair_group_count = requested_holes_per_row if explicit_holes_input else source_holes_per_row
                total_holes = (
                    pair_group_count * source_row_count
                    if source_row_count > 1
                    else pair_group_count
                )
            else:
                total_holes = requested_holes_per_row
            base["holes_per_row"] = max(1, total_holes)
            base["row_spacing"] = 0.0
            base["row_mode"] = "single_row_centerline"
        else:
            base["holes_per_row"] = requested_holes_per_row
            base["row_mode"] = (
                "staggered_double_row"
                if requested_pattern == "staggered_double_row"
                else "double_row"
            )
        bolt_dia = _float_at(plate_bolt_diameters, index, None)
        hole_clearance = _float_at(plate_hole_clearances, index, DEFAULT_HOLE_CLEARANCE)
        if bolt_dia is not None:
            base["bolt_dia"] = bolt_dia
            base["hole_clearance"] = hole_clearance
        if _has_values(plate_hole_diameters):
            base["diameter"] = _float_at(
                plate_hole_diameters,
                index,
                base.get("diameter", CODE_BASELINE_PLATE_HOLES["diameter"]),
            )
        elif bolt_dia is not None:
            base["diameter"] = bolt_dia + (hole_clearance or 0.0)
            base["diameter_source"] = "bolt_dia_plus_clearance"
        else:
            base["diameter"] = _float_at(
                plate_hole_diameters,
                index,
                base.get("diameter", CODE_BASELINE_PLATE_HOLES["diameter"]),
            )
        if requested_total_bolt_count is not None:
            actual_total = int(base["row_count"]) * int(base["holes_per_row"])
            base["requested_total_bolt_count"] = requested_total_bolt_count
            base["actual_total_bolt_count"] = actual_total
            base["bolt_count_alignment_ok"] = actual_total == requested_total_bolt_count
        if requested_row_count != 1:
            base["row_spacing"] = _float_at(
                plate_hole_row_spacings,
                index,
                base.get("row_spacing", CODE_BASELINE_PLATE_HOLES["row_spacing"]),
            )
        base["pitch"] = _float_at(
            plate_hole_pitches,
            index,
            base.get("pitch", CODE_BASELINE_PLATE_HOLES["pitch"]),
        )
        if base["row_mode"] == "staggered_double_row":
            base["stagger_offset"] = _float_at(
                plate_hole_stagger_offsets,
                index,
                0.5 * float(base.get("pitch") or 0.0),
            )
        else:
            base["stagger_offset"] = 0.0
        if any(
            _has_values(value)
            for value in (
                plate_hole_rows,
                plate_hole_patterns,
                plate_holes_per_row,
                plate_hole_diameters,
                plate_bolt_diameters,
                plate_hole_clearances,
                plate_total_hole_counts,
                plate_hole_row_spacings,
                plate_hole_pitches,
                plate_hole_stagger_offsets,
            )
        ):
            base["dimension_source"] = "override"
        result.append(base)
    return result


def _stiffener_specs_from_inputs(
    plane,
    target_plate_specs=None,
    baseplate_top_plane=None,
    plate_timber_widths=None,
    plate_timber_heights=None,
    bottom_face_mode="Perpendicular_to_grain",
    ref_with_stiffeners_brep=None,
    ref_stiffeners=None,
    stiffener_centers=None,
    stiffener_azimuths=None,
    stiffener_lengths=None,
    stiffener_widths=None,
    stiffener_heights=None,
    stiffener_low_heights=None,
    stiffener_thicknesses=None,
    stiffener_pair_axis_shift=None,
    stiffener_pair_from_point=None,
    stiffener_pair_to_point=None,
):
    explicit_target_overrides = any(
        _has_values(value)
        for value in (
            stiffener_centers,
            stiffener_azimuths,
            stiffener_lengths,
            stiffener_widths,
            stiffener_heights,
            stiffener_low_heights,
            stiffener_thicknesses,
        )
    )
    if target_plate_specs and not explicit_target_overrides:
        specs = _stiffener_specs_from_plate_targets(
            baseplate_top_plane or plane,
            target_plate_specs,
            plate_timber_widths=plate_timber_widths,
            plate_timber_heights=plate_timber_heights,
            bottom_face_mode=bottom_face_mode,
            default_thickness=_list_at(_flatten_values(stiffener_thicknesses), 0, None),
            stiffener_pair_axis_shift=stiffener_pair_axis_shift,
            stiffener_pair_from_point=stiffener_pair_from_point,
            stiffener_pair_to_point=stiffener_pair_to_point,
        )
    elif rg is not None and ref_stiffeners:
        specs = extract_oriented_brep_specs(ref_stiffeners, target_width=43.301, target_thickness=12.5)
    elif rg is not None and ref_with_stiffeners_brep is not None:
        source = _to_brep(_list_at(ref_with_stiffeners_brep, 0, ref_with_stiffeners_brep))
        specs = _combined_stiffener_specs_from_source(plane, source)
    else:
        specs = _default_stiffener_specs(plane)
    if not specs:
        specs = _default_stiffener_specs(plane)

    input_centers = _flatten_values(stiffener_centers)
    count = max(
        len(specs),
        len(input_centers),
        len(_flatten_values(stiffener_azimuths)),
        len(_flatten_values(stiffener_lengths)),
        len(_flatten_values(stiffener_widths)),
        len(_flatten_values(stiffener_heights)),
        len(_flatten_values(stiffener_low_heights)),
        len(_flatten_values(stiffener_thicknesses)),
    )
    result = []
    for index in range(count):
        base = dict(specs[index] if index < len(specs) else specs[-1])
        center = _point_from_value(_list_at(input_centers, index), base.get("center"))
        base["center"] = center
        base["azimuth_deg"] = _float_at(stiffener_azimuths, index, base.get("azimuth_deg", 0.0))
        base["length"] = _float_at(stiffener_lengths, index, base.get("length"))
        base["height"] = _float_at(stiffener_heights, index, base.get("height", base.get("width")))
        base["height"] = _float_at(stiffener_widths, index, base.get("height"))
        base["low_height"] = _float_at(stiffener_low_heights, index, base.get("low_height"))
        base["thickness"] = _float_at(stiffener_thicknesses, index, base.get("thickness"))
        result.append(base)
    return result


def _reference_baseplate_datum(ref_base_plate_brep):
    if rg is None or ref_base_plate_brep is None:
        return {}
    brep = _list_at(ref_base_plate_brep, 0, ref_base_plate_brep)
    if brep is None or not hasattr(brep, "GetBoundingBox"):
        return {}
    try:
        bbox = brep.GetBoundingBox(True)
        center = bbox.Center
        base_length = bbox.Max.X - bbox.Min.X
        base_width = bbox.Max.Y - bbox.Min.Y
        base_thickness = bbox.Max.Z - bbox.Min.Z
        bottom_plane = rg.Plane(
            rg.Point3d(center.X, center.Y, bbox.Min.Z),
            rg.Vector3d.XAxis,
            rg.Vector3d.YAxis,
        )
        return {
            "brep": _duplicate_geometry(brep) or brep,
            "bottom_plane": bottom_plane,
            "top_z": bbox.Max.Z,
            "bottom_z": bbox.Min.Z,
            "base_length": base_length,
            "base_width": base_width,
            "base_diameter": max(base_length, base_width),
            "base_thickness": base_thickness,
        }
    except Exception:
        return {}


def _build_base_outline(plane, base_length, base_width, base_shape="rectangular", base_diameter=None):
    if rg is None:
        return None
    if str(base_shape or "").strip().lower() == "circular":
        diameter = _coerce_float(base_diameter, max(base_length, base_width))
        return rg.Circle(plane, 0.5 * diameter).ToNurbsCurve()
    rectangle = rg.Rectangle3d(
        plane,
        rg.Interval(-0.5 * base_length, 0.5 * base_length),
        rg.Interval(-0.5 * base_width, 0.5 * base_width),
    )
    return rectangle.ToNurbsCurve()


def _build_hole_centers(
    plane,
    base_length,
    base_width,
    edge_spacing,
    hole_spacing,
    omit_center,
    base_shape="rectangular",
    base_diameter=None,
    hole_diameter=0.0,
    hole_count=4,
):
    if str(base_shape or "").strip().lower() == "circular":
        diameter = _coerce_float(base_diameter, max(base_length, base_width))
        min_edge = min(edge_spacing) if edge_spacing else 0.0
        pitch_radius = max(0.0, 0.5 * diameter - min_edge - 0.5 * hole_diameter)
        count = max(1, int(hole_count or 4))
        # Keep the default four-hole circular pattern on the plate-local
        # cardinal axes: +X, +Y, -X, -Y.
        start_angle = 0.0
        return [
            _point_on_plane(
                plane,
                (
                    pitch_radius * math.cos(start_angle + (2.0 * math.pi * index / count)),
                    pitch_radius * math.sin(start_angle + (2.0 * math.pi * index / count)),
                    0.0,
                ),
            )
            for index in range(count)
        ]

    left, right, bottom, top = edge_spacing
    spacing_x, spacing_y = hole_spacing
    x0 = -0.5 * base_length + left
    y0 = -0.5 * base_width + bottom
    xs = [x0, x0 + spacing_x, 0.5 * base_length - right]
    ys = [y0, y0 + spacing_y, 0.5 * base_width - top]

    centers = []
    for y_index, y in enumerate(ys):
        for x_index, x in enumerate(xs):
            if omit_center and x_index == 1 and y_index == 1:
                continue
            centers.append(_point_on_plane(plane, (x, y, 0.0)))
    return centers


def _build_hole_curve(center, plane, diameter):
    if rg is None:
        return None
    hole_plane = rg.Plane(center, plane.XAxis, plane.YAxis)
    return rg.Circle(hole_plane, 0.5 * diameter).ToNurbsCurve()


def _plate_hole_centers_for_spec(hole_spec, plate_spec):
    row_count = 2 if hole_spec.get("row_count") == 2 else 1
    holes_per_row = max(1, int(hole_spec.get("holes_per_row", 1)))
    pitch = hole_spec.get("pitch") or 0.0
    row_spacing = hole_spec.get("row_spacing") or 0.0
    stagger_offset = (
        hole_spec.get("stagger_offset") or 0.0
        if hole_spec.get("row_mode") == "staggered_double_row"
        else 0.0
    )
    center = hole_spec.get("center")
    if rg is None or center is None:
        return []

    plate_plane = _plate_plane(
        plate_spec["center"],
        plate_spec["azimuth_deg"],
        plate_spec["inclination_deg"],
    )
    row_offsets = [0.0] if row_count == 1 else [-0.5 * row_spacing, 0.5 * row_spacing]
    pattern_span = max(0, holes_per_row - 1) * pitch + stagger_offset
    centers = []
    for row_index, row_offset in enumerate(row_offsets):
        row_shift = stagger_offset if row_count == 2 and row_index == 1 else 0.0
        for index in range(holes_per_row):
            long_offset = index * pitch + row_shift - 0.5 * pattern_span
            point = rg.Point3d(center)
            point += plate_plane.XAxis * long_offset
            point += plate_plane.YAxis * row_offset
            centers.append(point)
    return centers


def _build_plate_hole_curve(center, plate_spec, diameter):
    if rg is None:
        return None
    plate_plane = _plate_plane(
        plate_spec["center"],
        plate_spec["azimuth_deg"],
        plate_spec["inclination_deg"],
    )
    return _build_hole_curve(center, plate_plane, diameter)


def _build_plate_hole_cutter(center, plate_spec, diameter, margin=0.0):
    if rg is None:
        return None
    plate_plane = _plate_plane(
        plate_spec["center"],
        plate_spec["azimuth_deg"],
        plate_spec["inclination_deg"],
    )
    depth = max(
        (plate_spec.get("thickness") or FOOTING_MIN_HOLE_CUTTER_DEPTH)
        * FOOTING_HOLE_CUTTER_THICKNESS_FACTOR,
        diameter * FOOTING_HOLE_CUTTER_DIAMETER_FACTOR,
        FOOTING_MIN_HOLE_CUTTER_DEPTH,
    )
    origin = rg.Point3d(center)
    origin -= plate_plane.ZAxis * (0.5 * depth)
    cutter_plane = rg.Plane(origin, plate_plane.XAxis, plate_plane.YAxis)
    cylinder = rg.Cylinder(rg.Circle(cutter_plane, 0.5 * diameter + margin), depth)
    return cylinder.ToBrep(True, True)


def _plate_hole_geometry(hole_specs, plate_specs, plug_margin=0.0):
    all_centers = []
    all_curves = []
    all_cutters = []
    per_plate_centers = []
    count = min(len(hole_specs or []), len(plate_specs or []))
    for index in range(count):
        hole_spec = hole_specs[index]
        plate_spec = plate_specs[index]
        centers = _plate_hole_centers_for_spec(hole_spec, plate_spec)
        per_plate_centers.append(centers)
        all_centers.extend(centers)
        diameter = hole_spec.get("diameter") or 0.0
        for center in centers:
            curve = _build_plate_hole_curve(center, plate_spec, diameter)
            cutter = _build_plate_hole_cutter(center, plate_spec, diameter, margin=plug_margin)
            if curve is not None:
                all_curves.append(curve)
            if cutter is not None:
                all_cutters.append(cutter)
    return all_centers, all_curves, all_cutters, per_plate_centers


def _plate_hole_pattern_diagnostics(hole_specs, plate_specs, per_plate_centers):
    diagnostics = []
    count = min(len(hole_specs or []), len(plate_specs or []), len(per_plate_centers or []))
    for index in range(count):
        hole_spec = hole_specs[index]
        plate_spec = plate_specs[index]
        centers = per_plate_centers[index]
        row_count = 2 if hole_spec.get("row_count") == 2 else 1
        holes_per_row = max(1, int(hole_spec.get("holes_per_row", 1)))
        expected_count = row_count * holes_per_row
        generated_count = len(centers)
        inside_count = None
        local_centers = []
        if rg is not None:
            plate_plane = _plate_plane(
                plate_spec["center"],
                plate_spec["azimuth_deg"],
                plate_spec["inclination_deg"],
            )
            half_length = 0.5 * float(plate_spec.get("length") or 0.0)
            half_width = 0.5 * float(plate_spec.get("width") or 0.0)
            inside_flags = []
            for center in centers:
                local = _point_local_in_plane(center, plate_plane)
                local_centers.append(local)
                inside_flags.append(
                    abs(local[0]) <= half_length + FOOTING_BOOLEAN_TOLERANCE
                    and abs(local[1]) <= half_width + FOOTING_BOOLEAN_TOLERANCE
                )
            inside_count = sum(1 for value in inside_flags if value)
        if generated_count != expected_count:
            status = "COUNT_MISMATCH"
        elif inside_count is not None and inside_count != expected_count:
            status = "OUTSIDE_PLATE"
        else:
            status = "OK"
        diagnostics.append(
            {
                "plate_index": index,
                "row_count": row_count,
                "holes_per_row": holes_per_row,
                "expected_count": expected_count,
                "generated_count": generated_count,
                "inside_plate_count": inside_count,
                "diameter": hole_spec.get("diameter"),
                "bolt_dia": hole_spec.get("bolt_dia"),
                "hole_clearance": hole_spec.get("hole_clearance"),
                "row_spacing": hole_spec.get("row_spacing"),
                "pitch": hole_spec.get("pitch"),
                "row_mode": hole_spec.get("row_mode"),
                "stagger_offset": hole_spec.get("stagger_offset"),
                "requested_total_bolt_count": hole_spec.get("requested_total_bolt_count"),
                "actual_total_bolt_count": hole_spec.get("actual_total_bolt_count"),
                "bolt_count_alignment_ok": hole_spec.get("bolt_count_alignment_ok"),
                "active_center_source": hole_spec.get("active_center_source"),
                "status": status,
                "local_centers": local_centers,
            }
        )
    return diagnostics


def _plane_payload(plane):
    if plane is None:
        return {}
    return {
        "origin": _debug_xyz(plane.Origin),
        "x_axis": _debug_xyz(plane.XAxis),
        "y_axis": _debug_xyz(plane.YAxis),
        "normal": _debug_xyz(plane.ZAxis),
    }


def _plate_frame_payloads(plate_specs):
    frames = []
    for spec in plate_specs or []:
        plane = _plate_plane(
            spec.get("center"),
            spec.get("azimuth_deg"),
            spec.get("inclination_deg"),
        )
        frame = {
            "member_id": spec.get("member_id"),
            "member_index": spec.get("member_index"),
        }
        frame.update(_plane_payload(plane))
        frames.append(frame)
    return frames


def _timber_bottom_face_refs_from_plate_specs(
    plate_specs,
    bottom_face_mode="Perpendicular_to_grain",
):
    """Expose resolved timber cut planes so downstream consumers can reuse them."""
    if rg is None:
        return []

    mode = _canonical_bottom_face_mode(bottom_face_mode)
    refs = []
    for spec in plate_specs or []:
        plate_plane = _plate_plane(
            spec.get("center"),
            spec.get("azimuth_deg"),
            spec.get("inclination_deg"),
        )
        heel = _plate_support_heel_point(spec)
        if plate_plane is None or heel is None:
            refs.append(
                {
                    "member_id": spec.get("member_id"),
                    "member_index": spec.get("member_index"),
                    "origin_source": "unresolved",
                }
            )
            continue

        heel_local = _point_local_in_plane(heel, plate_plane)
        collision_axis_distance = _coerce_float(
            spec.get("collision_axis_distance"),
            0.0,
        ) or 0.0
        collision_centerline_point = _point_on_plane(
            plate_plane,
            (
                heel_local[0] + collision_axis_distance,
                0.0,
                0.0,
            ),
        )
        effective_timber_bottom_z = _coerce_float(
            spec.get("effective_timber_bottom_z"),
            collision_centerline_point.Z,
        )
        if effective_timber_bottom_z is not None:
            collision_centerline_point.Z = float(effective_timber_bottom_z)

        width_axis = rg.Vector3d(plate_plane.ZAxis)
        if not width_axis.Unitize():
            width_axis = rg.Vector3d.YAxis
        if mode == "Parallel_to_ground":
            face_axis = rg.Vector3d(
                plate_plane.XAxis.X,
                plate_plane.XAxis.Y,
                0.0,
            )
            if not face_axis.Unitize():
                face_axis = rg.Vector3d.XAxis
            # Match the stiffener construction frame so downstream timber cuts
            # pivot to the same opposite side around the shared edge hinge.
            face_axis.Reverse()
        else:
            face_axis = rg.Vector3d(plate_plane.YAxis)
            if not face_axis.Unitize():
                face_axis = rg.Vector3d.ZAxis

        pivot_data = _bottom_face_pivot_data(
            spec,
            plate_plane,
            collision_centerline_point,
            effective_timber_bottom_z,
        )
        origin = pivot_data.get("point") or collision_centerline_point
        origin_source = pivot_data.get("source") or "collision_centerline_fallback"

        bottom_face_plane = rg.Plane(rg.Point3d(origin), width_axis, face_axis)
        ref = {
            "member_id": spec.get("member_id"),
            "member_index": spec.get("member_index"),
            "bottom_face_mode": mode,
            "origin_source": origin_source,
            "collision_centerline_point": _debug_xyz(collision_centerline_point),
            "effective_timber_bottom_z": spec.get("effective_timber_bottom_z"),
            "bottom_face_pivot_point": _debug_xyz(origin),
            "bottom_face_pivot_edge_name": pivot_data.get("edge_name"),
            "bottom_face_pivot_edge_start": _debug_xyz(pivot_data.get("edge_start")),
            "bottom_face_pivot_edge_end": _debug_xyz(pivot_data.get("edge_end")),
            "bottom_face_pivot_edge_parameter": pivot_data.get("edge_parameter"),
        }
        ref.update(_plane_payload(bottom_face_plane))
        refs.append(ref)
    return refs


def _largest_brep(breps):
    if not breps:
        return None
    def _score(brep):
        try:
            return brep.GetVolume()
        except Exception:
            try:
                return brep.GetBoundingBox(True).Diagonal.Length
            except Exception:
                return 0.0
    return max(breps, key=_score)


def _safe_brep_volume(brep):
    if brep is None:
        return None
    try:
        return float(brep.GetVolume())
    except Exception:
        return None


def _boolean_difference(subjects, cutters, tolerance):
    """Call RhinoCommon BooleanDifference with an overload-safe shape."""
    if rg is None:
        return None

    def _as_brep_list(value):
        if value is None:
            return []
        if isinstance(value, (list, tuple)):
            items = []
            for item in value:
                items.extend(_as_brep_list(item))
            return items
        return [value]

    subject_list = [brep for brep in _as_brep_list(subjects) if brep is not None]
    cutter_list = [brep for brep in _as_brep_list(cutters) if brep is not None]
    if not subject_list:
        return None
    if not cutter_list:
        return subject_list

    # Passing a single Brep plus a Python list asks RhinoPython to bind the
    # (Brep, Brep, tol) overload, then fail on the list. Supplying both sets as
    # collections should select the IEnumerable overload used for multi-cutter
    # cuts. Keep a single-Brep fallback for Rhino builds that still decline the
    # collection overload from CPython.
    try:
        return rg.Brep.CreateBooleanDifference(
            subject_list,
            cutter_list,
            tolerance,
        )
    except Exception:
        if len(subject_list) != 1:
            raise

    current = list(subject_list)
    for cutter in cutter_list:
        next_parts = []
        for subject in current:
            difference = rg.Brep.CreateBooleanDifference(
                subject,
                cutter,
                tolerance,
            )
            next_parts.extend([brep for brep in (difference or []) if brep is not None])
        current = next_parts
        if not current:
            break
    return current


def _repattern_full_plate_breps(full_plate_breps, source_hole_specs, target_hole_specs, plate_specs):
    if rg is None:
        return list(full_plate_breps or []), []

    rebuilt = []
    messages = []
    count = min(
        len(full_plate_breps or []),
        len(source_hole_specs or []),
        len(target_hole_specs or []),
        len(plate_specs or []),
    )
    for index in range(count):
        source_spec = source_hole_specs[index]
        target_spec = target_hole_specs[index]
        plate_spec = plate_specs[index]
        brep = _duplicate_geometry(full_plate_breps[index]) or full_plate_breps[index]

        _source_centers, _source_curves, plugs, _source_per_plate = _plate_hole_geometry(
            [source_spec],
            [plate_spec],
            plug_margin=FOOTING_PLATE_HOLE_PLUG_MARGIN,
        )
        _target_centers, _target_curves, cutters, _target_per_plate = _plate_hole_geometry(
            [target_spec],
            [plate_spec],
        )

        blank = brep
        if plugs:
            try:
                unioned = rg.Brep.CreateBooleanUnion([brep] + plugs, FOOTING_BOOLEAN_TOLERANCE)
                candidate = _largest_brep(unioned)
                if candidate is not None:
                    blank = candidate
                else:
                    messages.append("Plate {0}: source-hole fill union returned no Brep.".format(index))
            except Exception as exc:
                messages.append("Plate {0}: source-hole fill union failed: {1}".format(index, exc))

        result = blank
        if cutters:
            try:
                difference = _boolean_difference(blank, cutters, FOOTING_BOOLEAN_TOLERANCE)
                candidate = _largest_brep(difference)
                if candidate is not None:
                    result = candidate
                else:
                    messages.append("Plate {0}: plate-hole boolean difference returned no Brep.".format(index))
            except Exception as exc:
                messages.append("Plate {0}: plate-hole boolean difference failed: {1}".format(index, exc))
        rebuilt.append(result)

    if len(full_plate_breps or []) > count:
        rebuilt.extend(full_plate_breps[count:])
    return rebuilt, messages


def _cut_plate_holes_from_breps(plate_breps, target_hole_specs, plate_specs):
    if rg is None:
        return list(plate_breps or []), [], []

    rebuilt = []
    messages = []
    diagnostics = []
    count = min(
        len(plate_breps or []),
        len(target_hole_specs or []),
        len(plate_specs or []),
    )
    for index in range(count):
        target_spec = target_hole_specs[index]
        plate_spec = plate_specs[index]
        brep = _duplicate_geometry(plate_breps[index]) or plate_breps[index]
        _centers, _curves, cutters, _per_plate = _plate_hole_geometry(
            [target_spec],
            [plate_spec],
        )
        before_volume = _safe_brep_volume(brep)

        result = brep
        cut_mode = "none"
        cut_success_count = 0
        if cutters:
            try:
                difference = _boolean_difference(
                    brep,
                    cutters,
                    FOOTING_BOOLEAN_TOLERANCE,
                )
                candidate = _largest_brep(difference)
                if candidate is not None:
                    result = candidate
                    cut_mode = "batch"
                    cut_success_count = len(cutters)
                else:
                    messages.append("Plate body {0}: plate-hole boolean difference returned no Brep.".format(index))
            except Exception as exc:
                messages.append("Plate body {0}: plate-hole boolean difference failed: {1}".format(index, exc))
            if cut_mode == "none":
                sequential = brep
                for cutter_index, cutter in enumerate(cutters):
                    try:
                        difference = _boolean_difference(
                            sequential,
                            cutter,
                            FOOTING_BOOLEAN_TOLERANCE,
                        )
                        candidate = _largest_brep(difference)
                        if candidate is not None:
                            sequential = candidate
                            cut_success_count += 1
                        else:
                            messages.append(
                                "Plate body {0}: sequential hole {1} returned no Brep.".format(
                                    index,
                                    cutter_index,
                                )
                            )
                    except Exception as exc:
                        messages.append(
                            "Plate body {0}: sequential hole {1} failed: {2}".format(
                                index,
                                cutter_index,
                                exc,
                            )
                        )
                if cut_success_count:
                    result = sequential
                    cut_mode = "sequential"
        rebuilt.append(result)
        after_volume = _safe_brep_volume(result)
        diameter = float(target_spec.get("diameter") or 0.0)
        thickness = float(plate_spec.get("thickness") or 0.0)
        single_hole_volume = math.pi * (0.5 * diameter) ** 2 * thickness if diameter > 0.0 and thickness > 0.0 else None
        volume_delta = None if before_volume is None or after_volume is None else before_volume - after_volume
        estimated_effective_cut_count = (
            None
            if volume_delta is None or not single_hole_volume
            else volume_delta / single_hole_volume
        )
        diagnostics.append(
            {
                "plate_index": index,
                "cutter_count": len(cutters),
                "cut_mode": cut_mode,
                "cut_success_count": cut_success_count,
                "before_volume": before_volume,
                "after_volume": after_volume,
                "volume_delta": volume_delta,
                "single_hole_volume": single_hole_volume,
                "estimated_effective_cut_count": estimated_effective_cut_count,
            }
        )

    if len(plate_breps or []) > count:
        rebuilt.extend(plate_breps[count:])
    return rebuilt, messages, diagnostics


def _build_base_plate(
    plane,
    outline,
    hole_curves,
    base_length,
    base_width,
    base_thickness,
    base_shape="rectangular",
    base_diameter=None,
):
    if rg is None:
        return None
    if base_thickness and base_thickness > 0.0:
        if str(base_shape or "").strip().lower() == "circular":
            diameter = _coerce_float(base_diameter, max(base_length, base_width))
            base_brep = rg.Cylinder(rg.Circle(plane, 0.5 * diameter), base_thickness).ToBrep(True, True)
        else:
            box = rg.Box(
                plane,
                rg.Interval(-0.5 * base_length, 0.5 * base_length),
                rg.Interval(-0.5 * base_width, 0.5 * base_width),
                rg.Interval(0.0, base_thickness),
            )
            base_brep = box.ToBrep()
        cutters = []
        cutter_margin = max(2.0 * FOOTING_BOOLEAN_TOLERANCE, FOOTING_MIN_HOLE_CUTTER_DEPTH)
        cutter_depth = base_thickness + 2.0 * cutter_margin
        for curve in hole_curves or []:
            if curve is None:
                continue
            try:
                cutter_curve = curve.DuplicateCurve()
                cutter_curve.Transform(rg.Transform.Translation(plane.ZAxis * (-cutter_margin)))
                extrusion = rg.Extrusion.Create(cutter_curve, cutter_depth, True)
                cutter = extrusion.ToBrep() if extrusion is not None else None
                if cutter is not None:
                    cutters.append(cutter)
            except Exception:
                pass
        if cutters:
            try:
                difference = _boolean_difference(
                    base_brep,
                    cutters,
                    FOOTING_BOOLEAN_TOLERANCE,
                )
                candidate = _largest_brep(difference)
                if candidate is not None:
                    return candidate
            except Exception:
                pass
        return base_brep

    curves = [outline] + [curve for curve in hole_curves if curve is not None]
    try:
        breps = rg.Brep.CreatePlanarBreps(curves, 0.01)
        if breps:
            return breps[0]
    except Exception:
        pass
    return outline


def _merge_baseplate_with_web_plates(base_plate, web_plate_breps):
    if rg is None:
        pieces = [piece for piece in [base_plate] + list(web_plate_breps or []) if piece is not None]
        return pieces, [], False

    pieces = [piece for piece in [base_plate] + list(web_plate_breps or []) if piece is not None]
    if not pieces:
        return [], [], False
    if len(pieces) == 1:
        return pieces, [], True

    tolerances = [
        FOOTING_BOOLEAN_TOLERANCE,
        max(FOOTING_BOOLEAN_TOLERANCE * 2.0, 0.002),
        max(FOOTING_BOOLEAN_TOLERANCE * 5.0, 0.005),
        max(FOOTING_BOOLEAN_TOLERANCE * 10.0, 0.01),
    ]

    # Pass 1: direct all-at-once union with escalating tolerances.
    for tol in tolerances:
        try:
            unioned = rg.Brep.CreateBooleanUnion(pieces, tol)
            merged = [brep for brep in (unioned or []) if brep is not None]
            if merged:
                msg = "Baseplate/web-plate BooleanUnion succeeded at tol={0:.6f}.".format(tol)
                return merged, [msg], True
        except Exception:
            pass

    # Pass 2: iterative pairwise merge, which is often more robust than all-at-once.
    for tol in tolerances:
        current = [(_duplicate_geometry(p) or p) for p in pieces]
        changed = True
        while changed and len(current) > 1:
            changed = False
            next_parts = []
            used = [False] * len(current)
            for i in range(len(current)):
                if used[i]:
                    continue
                acc = current[i]
                used[i] = True
                for j in range(i + 1, len(current)):
                    if used[j]:
                        continue
                    try:
                        pair = rg.Brep.CreateBooleanUnion([acc, current[j]], tol)
                    except Exception:
                        pair = None
                    pair_merged = [brep for brep in (pair or []) if brep is not None]
                    if len(pair_merged) == 1:
                        acc = pair_merged[0]
                        used[j] = True
                        changed = True
                next_parts.append(acc)
            current = next_parts
        if len(current) == 1:
            msg = "Baseplate/web-plate pairwise BooleanUnion succeeded at tol={0:.6f}.".format(tol)
            return current, [msg], True

    # Pass 3: geometric join fallback (non-boolean). This can still output multiple parts.
    for tol in tolerances:
        try:
            joined = rg.Brep.JoinBreps(pieces, tol)
            joined_parts = [brep for brep in (joined or []) if brep is not None]
            if joined_parts:
                msg = "Baseplate/web-plate JoinBreps returned {0} part(s) at tol={1:.6f}.".format(
                    len(joined_parts), tol
                )
                return joined_parts, [msg], (len(joined_parts) == 1)
        except Exception:
            pass

    return pieces, ["Baseplate/web-plate merge failed; returning separate pieces."], False


def _subtract_web_plates_from_baseplate(base_plate, web_plate_breps):
    if rg is None:
        pieces = [piece for piece in [base_plate] + list(web_plate_breps or []) if piece is not None]
        return pieces, [], False

    web_plates = [brep for brep in (web_plate_breps or []) if brep is not None]
    if base_plate is None:
        return web_plates, ["Baseplate/web-plate BooleanDifference skipped: no baseplate Brep."], False
    if not web_plates:
        return [base_plate], [], True

    base_copy = _duplicate_geometry(base_plate) or base_plate
    cutters = [_duplicate_geometry(brep) or brep for brep in web_plates]
    try:
        difference = _boolean_difference(
            base_copy,
            cutters,
            FOOTING_BOOLEAN_TOLERANCE,
        )
        trimmed = [brep for brep in (difference or []) if brep is not None]
        candidate = _largest_brep(trimmed)
        if candidate is not None:
            return [candidate] + web_plates, [], True
        return [base_plate] + web_plates, [
            "Baseplate/web-plate BooleanDifference returned no Breps."
        ], False
    except Exception as exc:
        return [base_plate] + web_plates, [
            "Baseplate/web-plate BooleanDifference failed: {0}".format(exc)
        ], False


def _trim_web_plates_by_baseplate(base_plate, web_plate_breps, overlap=FOOTING_BOOLEAN_OVERLAP):
    if rg is None:
        pieces = [piece for piece in [base_plate] + list(web_plate_breps or []) if piece is not None]
        return pieces, [], False

    web_plates = [brep for brep in (web_plate_breps or []) if brep is not None]
    if base_plate is None:
        return web_plates, ["Web-plate trim skipped: no baseplate Brep."], False
    if not web_plates:
        return [base_plate], [], True

    # Use the real baseplate volume as the cutter so the trimmed web plates
    # finish flush on the actual top face. Earlier we lifted the cutter by a
    # small overlap to force intersection, which left a visible gap once the
    # heel profile itself crossed the baseplate plane.
    cutter = _duplicate_geometry(base_plate) or base_plate

    trimmed_web_plates = []
    messages = []
    success_count = 0
    for index, web_plate in enumerate(web_plates):
        source = _duplicate_geometry(web_plate) or web_plate
        try:
            difference = _boolean_difference(
                source,
                cutter,
                FOOTING_BOOLEAN_TOLERANCE,
            )
            candidate = _largest_brep(difference)
            if candidate is not None:
                trimmed_web_plates.append(candidate)
                success_count += 1
            else:
                trimmed_web_plates.append(web_plate)
                messages.append(
                    "Web plate {0}: baseplate trim returned no Brep; kept untrimmed plate.".format(index)
                )
        except Exception as exc:
            trimmed_web_plates.append(web_plate)
            messages.append(
                "Web plate {0}: baseplate trim failed: {1}".format(index, exc)
            )
    return [base_plate] + trimmed_web_plates, messages, success_count == len(web_plates)


def _build_plate_profile_curve(spec, plane):
    if rg is None or plane is None:
        return None
    length = float(spec.get("length") or 0.0)
    width = float(spec.get("width") or 0.0)
    radius = max(0.0, float(spec.get("heel_fillet_radius") or 0.0))
    if length <= 0.0 or width <= 0.0 or radius <= 0.0:
        return None

    local_heel = spec.get("support_heel_local") or (-0.5 * length, 0.5 * width, 0.0)
    heel_x_is_min = float(local_heel[0]) <= 0.0
    heel_y_is_max = float(local_heel[1]) >= 0.0
    x_min, x_max = -0.5 * length, 0.5 * length
    y_min, y_max = -0.5 * width, 0.5 * width
    max_radius = min(length, width) * 0.45
    radius = min(radius, max_radius)
    if radius <= FOOTING_BOOLEAN_TOLERANCE:
        return None

    def _pt(x, y):
        return _point_on_plane(plane, (x, y, 0.0))

    heel_x = x_min if heel_x_is_min else x_max
    heel_y = y_max if heel_y_is_max else y_min
    edge_x_dir = 1.0 if heel_x_is_min else -1.0
    base_x_dir = -float(plane.YAxis.Z)
    base_y_dir = float(plane.XAxis.Z)
    base_len = math.hypot(base_x_dir, base_y_dir)
    if base_len <= 1e-9:
        base_x_dir = 0.0
        base_y_dir = 1.0 if heel_y_is_max else -1.0
    else:
        base_x_dir /= base_len
        base_y_dir /= base_len
        desired_y_sign = 1.0 if heel_y_is_max else -1.0
        if base_y_dir * desired_y_sign < 0.0:
            base_x_dir *= -1.0
            base_y_dir *= -1.0

    start_local = (heel_x + edge_x_dir * radius, heel_y)
    end_local = (heel_x + base_x_dir * radius, heel_y + base_y_dir * radius)
    start_point = _pt(*start_local)
    end_point = _pt(*end_local)
    handle = max(0.55 * radius, 4.0 * FOOTING_BOOLEAN_TOLERANCE)

    def _slide_curve(start, end):
        start_tangent = _vector_on_plane(plane, (-edge_x_dir, 0.0, 0.0))
        end_tangent = _vector_on_plane(plane, (base_x_dir, base_y_dir, 0.0))
        if start_tangent is None or end_tangent is None:
            return None
        p1 = rg.Point3d(start)
        p1 += start_tangent * handle
        p2 = rg.Point3d(end)
        p2 -= end_tangent * handle
        try:
            return rg.NurbsCurve.Create(False, 3, [start, p1, p2, end])
        except Exception:
            return None

    edge_to_base_curve = _slide_curve(start_point, end_point)
    if edge_to_base_curve is None:
        return None

    def _curve_reversed(curve):
        duplicate = curve.DuplicateCurve()
        duplicate.Reverse()
        return duplicate

    if heel_x_is_min and heel_y_is_max:
        points = [
            _pt(x_min, y_min),
            _pt(x_max, y_min),
            _pt(x_max, y_max),
            start_point,
        ]
        transition = edge_to_base_curve
        tail = [end_point, _pt(x_min, y_min)]
    elif heel_x_is_min and not heel_y_is_max:
        points = [
            start_point,
            _pt(x_max, y_min),
            _pt(x_max, y_max),
            _pt(x_min, y_max),
            end_point,
        ]
        transition = _curve_reversed(edge_to_base_curve)
        tail = []
    elif not heel_x_is_min and heel_y_is_max:
        points = [
            _pt(x_min, y_min),
            _pt(x_max, y_min),
            end_point,
        ]
        transition = _curve_reversed(edge_to_base_curve)
        tail = [start_point, _pt(x_min, y_max), _pt(x_min, y_min)]
    else:
        points = [
            _pt(x_min, y_min),
            start_point,
        ]
        transition = edge_to_base_curve
        tail = [end_point, _pt(x_max, y_max), _pt(x_min, y_max), _pt(x_min, y_min)]

    segments = []
    for start, end in zip(points, points[1:]):
        segments.append(rg.LineCurve(start, end))
    segments.append(transition)
    for start, end in zip(tail, tail[1:]):
        segments.append(rg.LineCurve(start, end))
    if tail:
        segments.append(rg.LineCurve(tail[-1], points[0]))

    try:
        joined = rg.Curve.JoinCurves(segments, FOOTING_BOOLEAN_TOLERANCE)
        profile = joined[0] if joined else None
        if profile is not None and not profile.IsClosed:
            profile.MakeClosed(FOOTING_BOOLEAN_TOLERANCE)
        return profile if profile is not None and profile.IsClosed else None
    except Exception:
        return None


def _build_plate(spec):
    if rg is None:
        return None, None
    plane = _plate_plane(spec["center"], spec["azimuth_deg"], spec["inclination_deg"])
    long_axis = plane.XAxis
    profile = _build_plate_profile_curve(spec, plane)
    if profile is not None:
        try:
            extrusion_plane = rg.Plane(plane)
            extrusion_plane.Origin -= plane.ZAxis * (0.5 * spec["thickness"])
            xform = rg.Transform.PlaneToPlane(plane, extrusion_plane)
            profile.Transform(xform)
            extrusion = rg.Extrusion.Create(profile, spec["thickness"], True)
            if extrusion is not None:
                spec["plate_build_mode"] = "filleted_profile"
                axis = rg.Line(
                    plane.PointAt(-0.5 * spec["length"], 0.0),
                    plane.PointAt(0.5 * spec["length"], 0.0),
                )
                return extrusion.ToBrep(), axis
        except Exception:
            pass
    box = rg.Box(
        plane,
        rg.Interval(-0.5 * spec["length"], 0.5 * spec["length"]),
        rg.Interval(-0.5 * spec["width"], 0.5 * spec["width"]),
        rg.Interval(-0.5 * spec["thickness"], 0.5 * spec["thickness"]),
    )
    start = spec["center"] - long_axis * (0.5 * spec["length"])
    end = spec["center"] + long_axis * (0.5 * spec["length"])
    spec["plate_build_mode"] = "box_fallback"
    return box.ToBrep(), rg.Line(start, end).ToNurbsCurve()


def _build_full_plate(spec):
    # Fallback when the source filleted Brep is not wired. This stays coherent
    # as one Brep, but cannot reproduce the source fillet exactly without the
    # reference Brep.
    return _build_plate(
        {
            "center": spec["center"],
            "azimuth_deg": spec["azimuth_deg"],
            "inclination_deg": spec["inclination_deg"],
            "length": spec.get("full_length", spec.get("length")),
            "width": spec.get("width"),
            "thickness": spec.get("thickness"),
            "support_heel_local": spec.get("support_heel_local"),
            "heel_fillet_radius": spec.get("heel_fillet_radius"),
        }
    )[0]


def _transform_full_plate_breps(ref_full_plate_breps, source_specs, target_specs):
    refs = _flatten_values(ref_full_plate_breps)
    transformed = []
    for index, target in enumerate(target_specs):
        source_index = int(target.get("source_spec_index", index))
        if source_index >= len(refs) or source_index >= len(source_specs):
            continue
        source = source_specs[source_index]
        source_plane = source.get("plane")
        target_plane = _plate_plane(target["center"], target["azimuth_deg"], target["inclination_deg"])
        scale_x, scale_y, scale_z = _full_plate_transform_scales(source, target, index)
        brep = _transform_between_planes(refs[source_index], source_plane, target_plane, scale_x, scale_y, scale_z)
        if brep is not None:
            transformed.append(brep)
    return transformed


def _build_stiffener(spec):
    if rg is None:
        return None
    explicit_plane = spec.get("plane")
    box_dimensions = spec.get("box_dimensions")
    if explicit_plane is not None and box_dimensions:
        try:
            length, width, thickness = box_dimensions
            box = rg.Box(
                explicit_plane,
                rg.Interval(-0.5 * float(length), 0.5 * float(length)),
                rg.Interval(-0.5 * float(width), 0.5 * float(width)),
                rg.Interval(-0.5 * float(thickness), 0.5 * float(thickness)),
            )
            return box.ToBrep()
        except Exception:
            pass
    plane = _horizontal_plane(spec["center"], spec.get("azimuth_deg", 0.0))
    profile = spec.get("profile_points")
    length = spec.get("length") or 1.0
    height = spec.get("height") or 1.0
    thickness = spec.get("thickness") or 1.0

    if not profile:
        low_height = spec.get("low_height") or 0.5 * height
        bottom = -0.5 * height
        profile = (
            (-0.5 * length, bottom),
            (0.5 * length, bottom),
            (0.5 * length, bottom + height),
            (-0.5 * length, bottom + low_height),
        )

    ref_length = spec.get("reference_length") or spec.get("length") or 1.0
    ref_height = spec.get("reference_height") or spec.get("height") or 1.0
    sx = length / ref_length if ref_length else 1.0
    sz = height / ref_height if ref_height else 1.0
    half_t = 0.5 * thickness

    mesh = rg.Mesh()
    for x, z in profile:
        point = rg.Point3d(plane.Origin)
        point += plane.XAxis * (x * sx)
        point += plane.YAxis * (-half_t)
        point += plane.ZAxis * (z * sz)
        mesh.Vertices.Add(point)
    for x, z in profile:
        point = rg.Point3d(plane.Origin)
        point += plane.XAxis * (x * sx)
        point += plane.YAxis * half_t
        point += plane.ZAxis * (z * sz)
        mesh.Vertices.Add(point)

    count = len(profile)
    if count < 3:
        return None
    if count == 4:
        mesh.Faces.AddFace(0, 1, 2, 3)
        mesh.Faces.AddFace(7, 6, 5, 4)
    else:
        mesh.Faces.AddFace(0, 1, 2)
        mesh.Faces.AddFace(count + 2, count + 1, count)
    for index in range(count):
        next_index = (index + 1) % count
        mesh.Faces.AddFace(index, next_index, count + next_index, count + index)

    mesh.Normals.ComputeNormals()
    mesh.Compact()
    try:
        brep = rg.Brep.CreateFromMesh(mesh, True)
        if brep is not None:
            return brep
    except Exception:
        pass
    return mesh


def _transform_stiffener_breps(ref_stiffeners, source_specs, target_specs):
    refs = _flatten_values(ref_stiffeners)
    transformed = []
    for index, target in enumerate(target_specs):
        if index >= len(refs):
            break
        source = source_specs[index] if index < len(source_specs) else source_specs[-1]
        source_plane = source.get("plane") or _horizontal_plane(source["center"], source.get("azimuth_deg", 0.0))
        target_plane = _horizontal_plane(target["center"], target.get("azimuth_deg", 0.0))
        scale_x = (target.get("length") or source.get("length") or 1.0) / (source.get("length") or 1.0)
        scale_y = (target.get("width") or source.get("width") or 1.0) / (source.get("width") or 1.0)
        scale_z = (target.get("thickness") or source.get("thickness") or 1.0) / (source.get("thickness") or 1.0)
        brep = _transform_between_planes(refs[index], source_plane, target_plane, scale_x, scale_y, scale_z)
        if brep is not None:
            transformed.append(brep)
    return transformed


def _verify_breps(base_plate, full_plate_breps, stiffener_breps):
    report = {
        "checked": rg is not None,
        "base_valid": None,
        "full_plate_count": len(full_plate_breps or []),
        "full_plate_valid_count": 0,
        "stiffener_count": len(stiffener_breps or []),
        "stiffener_valid_count": 0,
        "join_count": None,
        "joined_is_valid": None,
        "joined_is_solid": None,
        "messages": [],
    }
    if rg is None:
        report["messages"].append("Rhino.Geometry is not available; run this component in Rhino/GH for Brep validation.")
        return report

    if base_plate is not None and hasattr(base_plate, "IsValid"):
        report["base_valid"] = bool(base_plate.IsValid)

    pieces = []
    for brep in full_plate_breps or []:
        valid = bool(getattr(brep, "IsValid", False))
        if valid:
            report["full_plate_valid_count"] += 1
        if brep is not None:
            pieces.append(brep)

    for brep in stiffener_breps or []:
        valid = bool(getattr(brep, "IsValid", False))
        if valid:
            report["stiffener_valid_count"] += 1
        if brep is not None:
            pieces.append(brep)

    if base_plate is not None and hasattr(base_plate, "Faces"):
        pieces.insert(0, base_plate)

    if pieces:
        try:
            joined = rg.Brep.JoinBreps(pieces, FOOTING_BOOLEAN_TOLERANCE)
            report["join_count"] = len(joined) if joined else 0
            if joined:
                report["joined_is_valid"] = all(bool(brep.IsValid) for brep in joined)
                report["joined_is_solid"] = all(bool(brep.IsSolid) for brep in joined)
        except Exception as exc:
            report["messages"].append("JoinBreps failed: {0}".format(exc))

    if report["full_plate_valid_count"] != report["full_plate_count"]:
        report["messages"].append("One or more full embedded plate Breps are invalid.")
    if report["stiffener_valid_count"] != report["stiffener_count"]:
        report["messages"].append("One or more stiffener Breps are invalid.")
    return report


def _hole_specs_from_centers(centers, diameter):
    return [
        {"source_index": index, "center": center, "diameter": diameter}
        for index, center in enumerate(centers)
    ]


def _build_metadata_handoff(
    base_length,
    base_width,
    base_thickness,
    hole_diameter,
    hole_specs,
    plate_specs,
    full_plate_specs,
    plate_hole_specs,
    plate_hole_centers,
    plate_hole_center_counts,
    plate_hole_pattern_diagnostics,
    stiffener_specs,
    verification,
    bottom_face_mode="Perpendicular_to_grain",
    baseplate_top_z=None,
    baseplate_top_source=None,
    baseplate_bottom_offset_z=0.0,
    plate_support_heel_z_values=None,
    plate_support_overlap=0.0,
    base_shape=None,
    base_diameter=None,
    heel_fillet_radius=None,
    timber_bottom_gap=None,
    min_timber_gap=None,
    sizing_source="code_baseline",
    applied_sizing_recommendations=None,
):
    first_plate = plate_specs[0] if plate_specs else {}
    first_plate_holes = plate_hole_specs[0] if plate_hole_specs else {}
    plate_hole_rows = first_plate_holes.get("row_count")
    plate_hole_row_spacing = first_plate_holes.get("row_spacing")
    plate_holes_per_row = int(first_plate_holes.get("holes_per_row") or 0)
    plate_total_hole_count = (
        int(plate_hole_rows or 0)
        * plate_holes_per_row
    )
    plate_depth = first_plate.get("width")
    edge_distance = None
    if plate_depth is not None and plate_hole_rows is not None and plate_hole_row_spacing is not None:
        edge_distance = max(
            0.0,
            0.5 * (
                float(plate_depth)
                - max(0, int(plate_hole_rows) - 1) * float(plate_hole_row_spacing)
            ),
        )
    annotation_dimensions = {
        "timber_width": first_plate.get("timber_width"),
        "timber_depth": first_plate.get("timber_height"),
        "plate_length": first_plate.get("length"),
        "plate_depth": first_plate.get("width"),
        "plate_thickness": first_plate.get("thickness"),
        "plate_hole_rows": first_plate_holes.get("row_count"),
        "plate_holes_per_row": first_plate_holes.get("holes_per_row"),
        "plate_hole_row_mode": first_plate_holes.get("row_mode"),
        "plate_hole_stagger_offset": first_plate_holes.get("stagger_offset"),
        "bolt_diameter": first_plate_holes.get("bolt_dia"),
        "bolt_hole_clearance": first_plate_holes.get("hole_clearance"),
        "bolt_hole_diameter": first_plate_holes.get("diameter"),
        "requested_total_bolt_count": first_plate_holes.get("requested_total_bolt_count"),
        "actual_total_bolt_count": plate_total_hole_count,
        "pitch_parallel": first_plate_holes.get("pitch"),
        "gage_perp": first_plate_holes.get("row_spacing"),
        "end_distance": first_plate.get("hole_end_distance"),
        "bottom_end_distance": first_plate.get("bottom_hole_end_distance"),
        "top_end_distance": first_plate.get("top_hole_end_distance"),
        "bottom_end_distance_multiplier": first_plate.get("bottom_end_distance_multiplier"),
        "edge_distance": edge_distance,
        "heel_fillet_radius": heel_fillet_radius,
        "base_diameter": base_diameter,
        "base_thickness": base_thickness,
        "baseplate_hole_diameter": hole_diameter,
        "timber_bottom_gap": timber_bottom_gap,
        "min_timber_gap": min_timber_gap,
    }
    layout_extents = {
        "base_shape": base_shape,
        "base_diameter": base_diameter,
        "base_length": base_length,
        "base_width": base_width,
        "base_thickness": base_thickness,
        "plate_count": len(plate_specs or []),
        "stiffener_count": len(stiffener_specs or []),
    }
    plate_frames = _plate_frame_payloads(plate_specs)
    timber_bottom_face_refs = _timber_bottom_face_refs_from_plate_specs(
        plate_specs,
        bottom_face_mode,
    )
    return {
        "geometry": {
            "base_length": base_length,
            "base_width": base_width,
            "base_shape": base_shape,
            "base_diameter": base_diameter,
            "base_thickness": base_thickness,
            "plate_length": first_plate.get("length"),
            "plate_depth": first_plate.get("width"),
            "plate_thickness": first_plate.get("thickness"),
            "plate_specs": plate_specs,
            "plate_frames": plate_frames,
            "full_plate_specs": full_plate_specs,
            "stiffener_specs": stiffener_specs,
            "timber_bottom_face_refs": timber_bottom_face_refs,
            "bottom_face_mode": bottom_face_mode,
            "baseplate_top_z": baseplate_top_z,
            "baseplate_top_source": baseplate_top_source,
            "baseplate_bottom_offset_z": baseplate_bottom_offset_z,
            "plate_support_heel_z_values": plate_support_heel_z_values or [],
            "plate_support_overlap": plate_support_overlap,
            "heel_fillet_radius": heel_fillet_radius,
            "timber_bottom_gap": timber_bottom_gap,
            "min_timber_gap": min_timber_gap,
            "sizing_source": sizing_source,
            "applied_sizing_recommendations": applied_sizing_recommendations or {},
        },
        "milling": {
            "baseplate_hole_diameter": hole_diameter,
            "baseplate_hole_specs": hole_specs,
            "bolt_dia": first_plate_holes.get("bolt_dia"),
            "hole_clearance": first_plate_holes.get("hole_clearance"),
            "bolt_hole_diameter": first_plate_holes.get("diameter"),
            "plate_hole_rows": first_plate_holes.get("row_count"),
            "plate_holes_per_row": first_plate_holes.get("holes_per_row"),
            "plate_hole_row_mode": first_plate_holes.get("row_mode"),
            "plate_hole_stagger_offset": first_plate_holes.get("stagger_offset"),
            "plate_total_hole_count": plate_total_hole_count,
            "requested_total_bolt_count": first_plate_holes.get("requested_total_bolt_count"),
            "actual_total_bolt_count": plate_total_hole_count,
            "bolt_count_alignment_ok": first_plate_holes.get("bolt_count_alignment_ok"),
            "plate_hole_specs": plate_hole_specs,
            "plate_hole_centers": [
                [_debug_xyz(point) for point in centers]
                for centers in (plate_hole_centers or [])
            ],
            "plate_hole_center_counts": list(plate_hole_center_counts or []),
            "plate_hole_pattern_diagnostics": list(plate_hole_pattern_diagnostics or []),
            "pitch_parallel": first_plate_holes.get("pitch"),
            "gage_perp": first_plate_holes.get("row_spacing"),
            "end_distance": first_plate.get("hole_end_distance"),
            "bottom_end_distance": first_plate.get("bottom_hole_end_distance"),
            "top_end_distance": first_plate.get("top_hole_end_distance"),
            "bottom_end_distance_multiplier": first_plate.get("bottom_end_distance_multiplier"),
            "edge_distance": edge_distance,
            "bolt_hole_dimension_source": first_plate_holes.get("dimension_source"),
        },
        "forces": {},
        "checks": {},
        "installation": {},
        "references": {
            "source_note": SOURCE_NOTE,
            "units": SOURCE_UNITS,
            "source_units": FOOTING_SOURCE_UNITS,
            "model_units": SOURCE_UNITS,
            "verification": verification,
        },
        "annotation": {
            "dimensions": annotation_dimensions,
            "critical_dimensions": {
                "units": SOURCE_UNITS,
                "input": dict(annotation_dimensions),
                "calculated": {},
                "resolved": dict(annotation_dimensions),
            },
            "checks": {},
            "labels": {},
        },
        "layout": {
            "extents": layout_extents,
            "dimension_summary": dict(annotation_dimensions),
            "critical_dimensions": {
                "units": SOURCE_UNITS,
                "input": dict(annotation_dimensions),
                "calculated": {},
                "resolved": dict(annotation_dimensions),
            },
            "named_views": ["NODE_TOP", "NODE_FRONT", "NODE_SECTION", "NODE_ISO"],
            "drawing_groups": ["top", "front", "section", "iso", "report", "title_block"],
            "report_sections": [
                "Part 1 timber/bolt checks",
                "Part 2 weld checks",
                "Part 3 screw-pier cap checks",
            ],
        },
    }


def _summarize(
    plate_specs,
    full_plate_specs,
    hole_specs,
    plate_hole_specs,
    include_stiffeners,
    base_length,
    base_width,
    verification=None,
):
    lines = [
        "Base Footing Param:",
        "Units: {0}".format(SOURCE_UNITS),
        "Base outline: {0:.1f} x {1:.1f}".format(base_length, base_width),
        "Embedded plates: {0}".format(len(plate_specs)),
        "Full filleted plate Breps: {0}".format(len(full_plate_specs)),
        "Baseplate anchor holes: {0}".format(len(hole_specs)),
        "Embedded plate hole patterns: {0}".format(len(plate_hole_specs)),
        "Stiffeners: {0}".format("included" if include_stiffeners else "not included"),
    ]
    for index, spec in enumerate(plate_specs):
        lines.append(
            "Plate {0}: L={1:.1f}, W={2:.1f}, T={3:.1f}, az={4:.2f} deg, alt={5:.2f} deg ({6}, member={7})".format(
                index,
                spec.get("length") or 0.0,
                spec.get("width") or 0.0,
                spec.get("thickness") or 0.0,
                spec.get("azimuth_deg") or 0.0,
                spec.get("altitude_deg", spec.get("inclination_deg")) or 0.0,
                spec.get("altitude_source") or "rhino_geometry",
                spec.get("member_index"),
            )
        )
    for index, spec in enumerate(plate_hole_specs):
        lines.append(
            "Plate holes {0}: rows={1}, per_row={2}, dia={3:.3f}, row_spacing={4:.3f}, pitch={5:.3f}".format(
                index,
                spec.get("row_count") or 0,
                spec.get("holes_per_row") or 0,
                spec.get("diameter") or 0.0,
                spec.get("row_spacing") or 0.0,
                spec.get("pitch") or 0.0,
            )
        )
    if verification:
        if verification.get("checked"):
            lines.append(
                "Validation: full plates {0}/{1} valid, stiffeners {2}/{3} valid, join_count={4}".format(
                    verification.get("full_plate_valid_count"),
                    verification.get("full_plate_count"),
                    verification.get("stiffener_valid_count"),
                    verification.get("stiffener_count"),
                    verification.get("join_count"),
                )
            )
        else:
            lines.append("Validation: run inside Rhino/GH for Brep validity and join checks.")
    return "\n".join(lines)


def base_footing_run(
    base_plane=None,
    ref_base_plate_brep=None,
    ref_plates=None,
    ref_full_plate_breps=None,
    ref_with_stiffeners_brep=None,
    ref_base_outline=None,
    ref_hole_curves=None,
    ref_stiffeners=None,
    timber_elements=None,
    plate_timber_indices=None,
    plate_member_indices=None,
    plate_member_ids=None,
    plate_orientation_sources=None,
    base_length=None,
    base_width=None,
    base_thickness=None,
    plate_centers=None,
    plate_azimuths=None,
    plate_altitudes=None,
    plate_inclinations=None,
    plate_lengths=None,
    plate_collision_zs=None,
    plate_collision_points=None,
    plate_timber_widths=None,
    plate_timber_heights=None,
    plate_full_lengths=None,
    plate_widths=None,
    plate_thicknesses=None,
    plate_full_centers=None,
    plate_hole_centers=None,
    plate_hole_rows=None,
    plate_hole_patterns=None,
    plate_holes_per_row=None,
    plate_hole_diameters=None,
    plate_bolt_diameters=None,
    plate_hole_clearances=None,
    plate_total_hole_counts=None,
    plate_hole_row_spacings=None,
    plate_hole_pitches=None,
    plate_hole_stagger_offsets=None,
    stiffener_centers=None,
    stiffener_azimuths=None,
    stiffener_lengths=None,
    stiffener_widths=None,
    stiffener_heights=None,
    stiffener_low_heights=None,
    stiffener_thicknesses=None,
    stiffener_pair_axis_shift=None,
    stiffener_pair_from_point=None,
    stiffener_pair_to_point=None,
    edge_spacing=None,
    hole_spacing=None,
    hole_diameter=None,
    hole_centers=None,
    baseplate_top_z=FOOTING_DEFAULT_BASEPLATE_TOP_Z,
    base_shape="circular",
    base_diameter=None,
    min_hole_edge_spacing=None,
    base_hole_count=4,
    sizing_recommendations=None,
    heel_fillet_radius=None,
    timber_bottom_gap=None,
    min_timber_gap=0.1,
    bottom_end_distance_multiplier=None,
    bottom_face_mode="Perpendicular_to_grain",
    include_stiffeners=None,
    omit_center_hole=True,
    enabled=True,
):
    if enabled is False:
        return {
            "summary_text": "Base Footing Param disabled.",
            "base_plate": None,
            "base_outline": None,
            "plate_breps": [],
            "full_plate_breps": [],
            "plate_axes": [],
            "hole_curves": [],
            "hole_centers": [],
            "hole_specs": [],
            "plate_hole_curves": [],
            "plate_hole_centers": [],
            "plate_hole_specs": [],
            "plate_hole_cutters": [],
            "stiffener_breps": [],
            "merged_footing_breps": [],
            "verification": {},
            "metadata": {
                "geometry": {},
                "milling": {},
                "forces": {},
                "checks": {},
                "installation": {},
                "references": {},
            },
            "params": {},
        }

    reference_baseplate = _reference_baseplate_datum(ref_base_plate_brep)
    plane = reference_baseplate.get("bottom_plane") or _as_plane(base_plane, reference_outline=ref_base_outline)
    explicit_plate_inputs = {
        "plate_lengths": _has_values(plate_lengths),
        "plate_widths": _has_values(plate_widths),
        "plate_thicknesses": _has_values(plate_thicknesses),
    }
    explicit_plate_hole_inputs = {
        "plate_hole_rows": _has_values(plate_hole_rows),
        "plate_hole_patterns": _has_values(plate_hole_patterns),
        "plate_holes_per_row": _has_values(plate_holes_per_row),
        "plate_hole_diameters": _has_values(plate_hole_diameters),
        "plate_bolt_diameters": _has_values(plate_bolt_diameters),
        "plate_hole_clearances": _has_values(plate_hole_clearances),
        "plate_total_hole_counts": _has_values(plate_total_hole_counts),
        "plate_hole_row_spacings": _has_values(plate_hole_row_spacings),
        "plate_hole_pitches": _has_values(plate_hole_pitches),
        "plate_hole_stagger_offsets": _has_values(plate_hole_stagger_offsets),
    }
    normalized_sizing_recommendations = _scale_sizing_recommendations(sizing_recommendations)
    applied_sizing_recommendations = {}
    if normalized_sizing_recommendations:
        if normalized_sizing_recommendations.get("plate_length") is not None and not explicit_plate_inputs["plate_lengths"]:
            plate_lengths = normalized_sizing_recommendations["plate_length"]
            applied_sizing_recommendations["plate_length"] = plate_lengths
        if normalized_sizing_recommendations.get("plate_width") is not None and not explicit_plate_inputs["plate_widths"]:
            plate_widths = normalized_sizing_recommendations["plate_width"]
            applied_sizing_recommendations["plate_width"] = plate_widths
        if normalized_sizing_recommendations.get("plate_thickness") is not None and not explicit_plate_inputs["plate_thicknesses"]:
            plate_thicknesses = normalized_sizing_recommendations["plate_thickness"]
            applied_sizing_recommendations["plate_thickness"] = plate_thicknesses
        if normalized_sizing_recommendations.get("rows") is not None and not explicit_plate_hole_inputs["plate_hole_rows"]:
            plate_hole_rows = normalized_sizing_recommendations["rows"]
            applied_sizing_recommendations["rows"] = plate_hole_rows
        if normalized_sizing_recommendations.get("holes_per_row") is not None and not explicit_plate_hole_inputs["plate_holes_per_row"]:
            plate_holes_per_row = normalized_sizing_recommendations["holes_per_row"]
            applied_sizing_recommendations["holes_per_row"] = plate_holes_per_row
        sizing_total_bolt_count = (
            normalized_sizing_recommendations.get("total_bolt_count")
            or normalized_sizing_recommendations.get("actual_total_bolt_count")
            or normalized_sizing_recommendations.get("recommended_total_bolt_count")
        )
        if sizing_total_bolt_count is not None and not explicit_plate_hole_inputs["plate_total_hole_counts"]:
            plate_total_hole_counts = sizing_total_bolt_count
            applied_sizing_recommendations["total_bolt_count"] = plate_total_hole_counts
        if normalized_sizing_recommendations.get("bolt_dia") is not None and not explicit_plate_hole_inputs["plate_bolt_diameters"]:
            plate_bolt_diameters = normalized_sizing_recommendations["bolt_dia"]
            applied_sizing_recommendations["bolt_dia"] = plate_bolt_diameters
        if normalized_sizing_recommendations.get("hole_clearance") is not None and not explicit_plate_hole_inputs["plate_hole_clearances"]:
            plate_hole_clearances = normalized_sizing_recommendations["hole_clearance"]
            applied_sizing_recommendations["hole_clearance"] = plate_hole_clearances
        if normalized_sizing_recommendations.get("hole_pattern") is not None and not explicit_plate_hole_inputs["plate_hole_patterns"]:
            plate_hole_patterns = normalized_sizing_recommendations["hole_pattern"]
            applied_sizing_recommendations["hole_pattern"] = plate_hole_patterns
        if normalized_sizing_recommendations.get("bolt_hole_dia") is not None and not explicit_plate_hole_inputs["plate_hole_diameters"]:
            plate_hole_diameters = normalized_sizing_recommendations["bolt_hole_dia"]
            applied_sizing_recommendations["bolt_hole_dia"] = plate_hole_diameters
        if normalized_sizing_recommendations.get("gage_perp") is not None and not explicit_plate_hole_inputs["plate_hole_row_spacings"]:
            plate_hole_row_spacings = normalized_sizing_recommendations["gage_perp"]
            applied_sizing_recommendations["gage_perp"] = plate_hole_row_spacings
        if normalized_sizing_recommendations.get("pitch_parallel") is not None and not explicit_plate_hole_inputs["plate_hole_pitches"]:
            plate_hole_pitches = normalized_sizing_recommendations["pitch_parallel"]
            applied_sizing_recommendations["pitch_parallel"] = plate_hole_pitches
        if normalized_sizing_recommendations.get("stagger_offset") is not None and not explicit_plate_hole_inputs["plate_hole_stagger_offsets"]:
            plate_hole_stagger_offsets = normalized_sizing_recommendations["stagger_offset"]
            applied_sizing_recommendations["stagger_offset"] = plate_hole_stagger_offsets
    explicit_heel_fillet_radius = _coerce_float(heel_fillet_radius, None)
    governing_heel_fillet_radius = PROJECT_MIN_HEEL_FILLET_RADIUS
    if normalized_sizing_recommendations:
        code_min_corner_radius = _coerce_float(
            normalized_sizing_recommendations.get("corner_radius_code_min"),
            None,
        )
        project_min_corner_radius = _coerce_float(
            normalized_sizing_recommendations.get("corner_radius_project_min"),
            PROJECT_MIN_HEEL_FILLET_RADIUS,
        )
        governing_corner_radius = _coerce_float(
            normalized_sizing_recommendations.get("corner_radius_governing_min"),
            None,
        )
        if governing_corner_radius is None:
            governing_candidates = [
                value
                for value in (code_min_corner_radius, project_min_corner_radius)
                if value is not None
            ]
            if governing_candidates:
                governing_corner_radius = max(governing_candidates)
        if governing_corner_radius is not None:
            governing_heel_fillet_radius = max(
                PROJECT_MIN_HEEL_FILLET_RADIUS,
                governing_corner_radius,
            )
    preferred_heel_fillet_radius = max(
        PROJECT_DEFAULT_HEEL_FILLET_RADIUS,
        governing_heel_fillet_radius,
    )
    resolved_heel_fillet_radius = explicit_heel_fillet_radius
    if resolved_heel_fillet_radius is None and normalized_sizing_recommendations:
        resolved_heel_fillet_radius = _coerce_float(
            normalized_sizing_recommendations.get("corner_radius"),
            None,
        )
        if resolved_heel_fillet_radius is not None:
            applied_sizing_recommendations["corner_radius"] = resolved_heel_fillet_radius
    if resolved_heel_fillet_radius is None:
        resolved_heel_fillet_radius = preferred_heel_fillet_radius
    else:
        resolved_heel_fillet_radius = max(
            resolved_heel_fillet_radius,
            governing_heel_fillet_radius,
        )
        if explicit_heel_fillet_radius is None:
            resolved_heel_fillet_radius = max(
                resolved_heel_fillet_radius,
                preferred_heel_fillet_radius,
            )
    resolved_min_timber_gap = _coerce_float(min_timber_gap, 0.1)
    resolved_timber_bottom_gap = _coerce_float(timber_bottom_gap, resolved_min_timber_gap)
    has_explicit_sizing_overrides = bool(
        any(explicit_plate_inputs.values()) or any(explicit_plate_hole_inputs.values())
    )
    if applied_sizing_recommendations and has_explicit_sizing_overrides:
        sizing_source = "engineering_sizing_recommendations_with_overrides"
    elif applied_sizing_recommendations:
        sizing_source = "engineering_sizing_recommendations"
    elif has_explicit_sizing_overrides:
        sizing_source = "explicit_override"
    else:
        sizing_source = "code_baseline"
    base_length_value = _coerce_float(
        base_length,
        reference_baseplate.get("base_length"),
    )
    base_width_value = _coerce_float(
        base_width,
        reference_baseplate.get("base_width"),
    )
    base_diameter_value = _coerce_float(
        base_diameter,
        reference_baseplate.get("base_diameter"),
    )
    base_thickness = _coerce_float(
        base_thickness,
        reference_baseplate.get("base_thickness", REFERENCE["base_thickness"]),
    )
    hole_diameter_value = _coerce_float(hole_diameter, None)
    include_stiffeners_override = _coerce_optional_bool(include_stiffeners)
    if include_stiffeners_override is None:
        include_stiffeners = bool(ref_stiffeners or ref_with_stiffeners_brep)
    else:
        include_stiffeners = include_stiffeners_override
    omit_center = _coerce_bool(omit_center_hole, True)
    if min_hole_edge_spacing is not None:
        uniform_edge_spacing = _coerce_float(min_hole_edge_spacing, REFERENCE["edge_spacing"][0])
        edge_spacing_tuple = (
            uniform_edge_spacing,
            uniform_edge_spacing,
            uniform_edge_spacing,
            uniform_edge_spacing,
        )
    else:
        edge_spacing_tuple = _edge_spacing_tuple(edge_spacing)
    hole_spacing_pair = _spacing_pair(hole_spacing)

    if rg is not None and ref_base_outline is not None and (base_length_value is None or base_width_value is None):
        outline = _list_at(ref_base_outline, 0, ref_base_outline)
        if outline is not None and hasattr(outline, "GetBoundingBox"):
            bbox = outline.GetBoundingBox(True)
            if base_length_value is None:
                base_length_value = bbox.Max.X - bbox.Min.X
            if base_width_value is None:
                base_width_value = bbox.Max.Y - bbox.Min.Y

    normalized_base_shape = str(base_shape or REFERENCE.get("base_shape") or "rectangular").strip().lower()
    if reference_baseplate:
        normalized_base_shape = "reference"
    base_diameter_value = _coerce_float(base_diameter_value, REFERENCE.get("base_diameter"))
    if normalized_base_shape == "circular":
        base_length = _coerce_float(base_length_value, base_diameter_value)
        base_width = _coerce_float(base_width_value, base_diameter_value)
    else:
        base_length = _coerce_float(base_length_value, REFERENCE["base_length"])
        base_width = _coerce_float(base_width_value, REFERENCE["base_width"])

    plate_specs = _plate_specs_from_inputs(
        plane,
        reference_plates=ref_plates,
        timber_elements=timber_elements,
        plate_timber_indices=plate_timber_indices,
        plate_member_indices=plate_member_indices,
        plate_member_ids=plate_member_ids,
        plate_orientation_sources=plate_orientation_sources,
        plate_centers=plate_centers,
        plate_azimuths=plate_azimuths,
        plate_altitudes=plate_altitudes,
        plate_inclinations=plate_inclinations,
        plate_lengths=plate_lengths,
        plate_widths=plate_widths,
        plate_thicknesses=plate_thicknesses,
    )
    plate_specs = _canonicalize_straight_plate_heel_specs(plate_specs)
    requested_baseplate_top_z = _coerce_float(baseplate_top_z, None)
    if reference_baseplate.get("top_z") is not None:
        requested_baseplate_top_z = reference_baseplate["top_z"]
    if requested_baseplate_top_z is not None:
        plate_specs = _shift_plate_specs_to_heel_z(
            plate_specs,
            requested_baseplate_top_z,
        )
    full_plate_specs = _full_plate_specs_from_inputs(
        plane,
        ref_full_plate_breps,
        plate_specs,
        plate_full_centers=plate_full_centers,
        plate_full_lengths=plate_full_lengths,
    )
    plate_support_heel_points = _plate_support_heel_points(plate_specs)
    plate_support_heel_z_values = [point.Z for point in plate_support_heel_points]
    fixed_baseplate_top_z = requested_baseplate_top_z
    if reference_baseplate.get("top_z") is not None and rg is not None and plane is not None:
        resolved_baseplate_top_z = reference_baseplate["top_z"]
        baseplate_top_source = "reference_base_plate"
        baseplate_bottom_offset_z = 0.0
    elif fixed_baseplate_top_z is not None and rg is not None and plane is not None:
        resolved_baseplate_top_z = fixed_baseplate_top_z
        baseplate_top_source = "fixed_input"
        baseplate_bottom_offset_z = resolved_baseplate_top_z - base_thickness - plane.Origin.Z
    elif plate_support_heel_z_values and rg is not None and plane is not None:
        resolved_baseplate_top_z = max(plate_support_heel_z_values) + FOOTING_BOOLEAN_OVERLAP
        baseplate_top_source = "plate_support_heel"
        baseplate_bottom_offset_z = resolved_baseplate_top_z - base_thickness - plane.Origin.Z
    else:
        resolved_baseplate_top_z = (plane.Origin.Z + base_thickness) if rg is not None and plane is not None else None
        baseplate_top_source = "fallback"
        baseplate_bottom_offset_z = 0.0
    baseplate_plane = _translated_plane_world_z(plane, baseplate_bottom_offset_z)

    source_full_specs = []
    if rg is not None and ref_full_plate_breps:
        source_full_specs = extract_oriented_brep_specs(
            ref_full_plate_breps,
            target_width=140.0,
            target_thickness=10.0,
        )
        for index, source in enumerate(source_full_specs):
            if index < len(FULL_PLATE_REFERENCE):
                source["straight_length"] = FULL_PLATE_REFERENCE[index]["straight_length"]
    if not source_full_specs:
        source_full_specs = _default_full_plate_specs(plane)

    source_plate_hole_specs = []
    if rg is not None and ref_full_plate_breps:
        source_plate_hole_specs = extract_plate_hole_specs(
            ref_full_plate_breps,
            source_full_specs,
        )
    if not source_plate_hole_specs:
        source_plate_hole_specs = _default_plate_hole_specs(plane)
    transformed_source_plate_hole_specs = _transformed_source_plate_hole_specs(
        source_plate_hole_specs,
        source_full_specs,
        full_plate_specs,
    )
    recommended_row_count = _coerce_float(
        normalized_sizing_recommendations.get("rows")
        if normalized_sizing_recommendations
        else None,
        None,
    )
    collapse_single_row_from_pairs = bool(
        explicit_plate_hole_inputs["plate_hole_rows"]
        and int(recommended_row_count or CODE_BASELINE_PLATE_HOLES["row_count"]) != 1
    )
    if not normalized_sizing_recommendations and explicit_plate_hole_inputs["plate_hole_rows"]:
        collapse_single_row_from_pairs = True
    plate_hole_specs = _plate_hole_specs_from_inputs(
        plane,
        source_plate_hole_specs,
        source_full_specs,
        full_plate_specs,
        plate_hole_centers=plate_hole_centers,
        plate_hole_rows=plate_hole_rows,
        plate_hole_patterns=plate_hole_patterns,
        plate_holes_per_row=plate_holes_per_row,
        plate_hole_diameters=plate_hole_diameters,
        plate_bolt_diameters=plate_bolt_diameters,
        plate_hole_clearances=plate_hole_clearances,
        plate_total_hole_counts=plate_total_hole_counts,
        plate_hole_row_spacings=plate_hole_row_spacings,
        plate_hole_pitches=plate_hole_pitches,
        plate_hole_stagger_offsets=plate_hole_stagger_offsets,
        collapse_single_row_from_pairs=collapse_single_row_from_pairs,
    )
    collision_z_values = [_coerce_float(value, None) for value in _flatten_values(plate_collision_zs)]
    collision_point_values = [
        _point_from_value(value)
        for value in _flatten_values(plate_collision_points)
    ]
    if collision_z_values:
        for index, spec in enumerate(plate_specs):
            collision_z = _list_at(collision_z_values, index, None)
            collision_point = _list_at(collision_point_values, index, None)
            spec["collision_z"] = collision_z
            spec["collision_point_input"] = collision_point
            if collision_z is None or resolved_baseplate_top_z is None:
                spec["collision_height_from_baseplate"] = None
                continue
            raw_collision_gap = float(collision_z) - float(resolved_baseplate_top_z)
            effective_collision_gap = max(raw_collision_gap, resolved_timber_bottom_gap)
            effective_collision_z = float(resolved_baseplate_top_z) + effective_collision_gap
            spec["raw_collision_gap"] = raw_collision_gap
            spec["effective_collision_gap"] = effective_collision_gap
            spec["effective_collision_z"] = effective_collision_z
            spec["effective_timber_bottom_z"] = effective_collision_z
            spec["collision_height_from_baseplate"] = effective_collision_gap
            if rg is not None and collision_point is not None:
                collision_point_at_effective_z = rg.Point3d(collision_point)
                collision_point_at_effective_z.Z = effective_collision_z
                spec["collision_point_at_effective_z"] = collision_point_at_effective_z
            sin_altitude = abs(math.sin(math.radians(float(spec.get("inclination_deg") or 0.0))))
            if sin_altitude > 1e-9:
                collision_axis_distance = effective_collision_gap / sin_altitude
                active_hole_spec = plate_hole_specs[index] if index < len(plate_hole_specs) else {}
                hole_count = max(1, int(active_hole_spec.get("holes_per_row") or 1))
                pitch_value = float(active_hole_spec.get("pitch") or CODE_BASELINE_PLATE_HOLES["pitch"])
                baseline_pattern_length = float(spec.get("length") or 0.0)
                end_distance_value = _coerce_float(
                    normalized_sizing_recommendations.get("end_distance")
                    if normalized_sizing_recommendations
                    else None,
                    None,
                )
                if end_distance_value is None or end_distance_value <= 0.0:
                    end_distance_value = float(CODE_BASELINE_PLATE_HOLES["end_distance"])
                requested_bottom_end_distance_multiplier = _coerce_float(
                    normalized_sizing_recommendations.get("bottom_end_distance_multiplier")
                    if normalized_sizing_recommendations
                    else bottom_end_distance_multiplier,
                    1.05,
                )
                bottom_end_distance_multiplier = (
                    requested_bottom_end_distance_multiplier
                    if bottom_face_mode == "Parallel_to_ground"
                    else 1.0
                )
                bottom_end_distance = _coerce_float(
                    normalized_sizing_recommendations.get("bottom_end_distance")
                    if normalized_sizing_recommendations
                    else None,
                    None,
                )
                if bottom_end_distance is None or bottom_end_distance <= 0.0:
                    bottom_end_distance = end_distance_value * bottom_end_distance_multiplier
                top_end_distance = _coerce_float(
                    normalized_sizing_recommendations.get("top_end_distance")
                    if normalized_sizing_recommendations
                    else None,
                    None,
                )
                if top_end_distance is None or top_end_distance <= 0.0:
                    top_end_distance = end_distance_value
                stagger_offset = (
                    float(active_hole_spec.get("stagger_offset") or 0.0)
                    if active_hole_spec.get("row_mode") == "staggered_double_row"
                    else 0.0
                )
                pattern_span = (
                    max(0, hole_count - 1) * float(pitch_value or 0.0)
                    + stagger_offset
                )
                code_pattern_length = (
                    bottom_end_distance
                    + pattern_span
                    + top_end_distance
                )
                effective_plate_length = collision_axis_distance + code_pattern_length
                first_hole_axis_offset = collision_axis_distance + bottom_end_distance
                last_hole_axis_offset = first_hole_axis_offset + pattern_span
                spec["collision_axis_distance"] = collision_axis_distance
                spec["code_pattern_length"] = code_pattern_length
                spec["hole_end_distance"] = end_distance_value
                spec["bottom_hole_end_distance"] = bottom_end_distance
                spec["top_hole_end_distance"] = top_end_distance
                spec["bottom_end_distance_multiplier"] = bottom_end_distance_multiplier
                spec["hole_pattern_span"] = pattern_span
                spec["hole_stagger_offset"] = stagger_offset
                spec["effective_plate_length"] = effective_plate_length
                spec["required_heel_to_tip_distance"] = effective_plate_length
                spec["first_hole_axis_offset"] = first_hole_axis_offset
                spec["last_hole_axis_offset"] = last_hole_axis_offset
                spec["top_end_distance_from_last_hole"] = effective_plate_length - last_hole_axis_offset
                if effective_plate_length > float(spec.get("length") or 0.0):
                    spec["length"] = effective_plate_length
                    spec["length_source"] = "collision_gap_plus_code_pattern"
    if resolved_heel_fillet_radius is not None:
        for spec in plate_specs:
            spec["heel_fillet_radius"] = resolved_heel_fillet_radius
    if resolved_baseplate_top_z is not None:
        plate_specs = _shift_plate_specs_to_heel_z(
            plate_specs,
            resolved_baseplate_top_z,
        )
        plate_support_heel_points = _plate_support_heel_points(plate_specs)
        plate_support_heel_z_values = [point.Z for point in plate_support_heel_points]
    if rg is not None:
        for spec in plate_specs:
            collision_point_at_effective_z = spec.get("collision_point_at_effective_z")
            if collision_point_at_effective_z is None:
                continue
            plate_plane = _plate_plane(
                spec["center"],
                spec["azimuth_deg"],
                spec["inclination_deg"],
            )
            if plate_plane is None:
                continue
            projected_collision_point = plate_plane.ClosestPoint(
                collision_point_at_effective_z
            )
            spec["projected_collision_point"] = projected_collision_point
            edge_projected_point, edge_name = _nearest_plate_edge_point(
                spec,
                plate_plane,
                projected_collision_point,
            )
            spec["edge_projected_collision_point"] = edge_projected_point
            spec["edge_projected_collision_name"] = edge_name
    full_plate_specs = _full_plate_specs_from_inputs(
        plane,
        ref_full_plate_breps,
        plate_specs,
        plate_full_centers=plate_full_centers,
        plate_full_lengths=plate_full_lengths,
    )
    plate_hole_warnings = []
    for index, spec in enumerate(plate_hole_specs):
        if spec.get("row_count") == 2 and (spec.get("row_spacing") or 0.0) <= 0.0:
            plate_hole_warnings.append(
                "Plate {0}: two-row hole pattern has zero row spacing.".format(index)
            )
    plate_hole_override_connected = bool(
        _has_values(plate_hole_centers)
        or any(explicit_plate_hole_inputs.values())
    )
    plate_hole_sizing_applied = any(
        key in applied_sizing_recommendations
        for key in (
            "rows",
            "holes_per_row",
            "bolt_hole_dia",
            "gage_perp",
            "pitch_parallel",
        )
    )
    if plate_hole_sizing_applied and not plate_hole_override_connected:
        for spec in plate_hole_specs:
            spec["dimension_source"] = "engineering_sizing_recommendations"
    full_plate_hole_specs = plate_hole_specs
    active_plate_hole_specs = _active_plate_hole_specs_for_web_plates(
        full_plate_hole_specs,
        plate_specs,
        preserve_input_centers=_has_values(plate_hole_centers),
    )
    plate_body_scaled = _full_plate_transform_changed(source_full_specs, full_plate_specs)
    plate_hole_pattern_changed = _plate_hole_pattern_changed(
        transformed_source_plate_hole_specs,
        full_plate_hole_specs,
    )

    supplied_hole_centers = [
        _point_from_value(value)
        for value in _flatten_values(hole_centers)
    ]
    supplied_hole_centers = [point for point in supplied_hole_centers if point is not None]

    if supplied_hole_centers:
        hole_diameter = _coerce_float(hole_diameter_value, REFERENCE["hole_diameter"])
        centers = supplied_hole_centers
        hole_specs = _hole_specs_from_centers(centers, hole_diameter)
    elif rg is not None and ref_hole_curves:
        hole_specs = extract_hole_specs(ref_hole_curves)
        centers = [spec["center"] for spec in hole_specs]
        if hole_specs and hole_diameter_value is None:
            hole_diameter = hole_specs[0].get("diameter", REFERENCE["hole_diameter"])
        else:
            hole_diameter = _coerce_float(hole_diameter_value, REFERENCE["hole_diameter"])
        for spec in hole_specs:
            spec["diameter"] = hole_diameter
    else:
        hole_diameter = _coerce_float(hole_diameter_value, REFERENCE["hole_diameter"])
        centers = _build_hole_centers(
            baseplate_plane,
            base_length,
            base_width,
            edge_spacing_tuple,
            hole_spacing_pair,
            omit_center,
            base_shape=normalized_base_shape,
            base_diameter=base_diameter_value,
            hole_diameter=hole_diameter,
            hole_count=base_hole_count,
        )
        hole_specs = _hole_specs_from_centers(centers, hole_diameter)

    base_outline = _build_base_outline(
        baseplate_plane,
        base_length,
        base_width,
        base_shape=normalized_base_shape,
        base_diameter=base_diameter_value,
    )
    hole_curves = [_build_hole_curve(center, baseplate_plane, hole_diameter) for center in centers]
    base_plate = reference_baseplate.get("brep")
    if base_plate is None:
        base_plate = _build_base_plate(
            baseplate_plane,
            base_outline,
            hole_curves,
            base_length,
            base_width,
            base_thickness,
            base_shape=normalized_base_shape,
            base_diameter=base_diameter_value,
        )

    plate_breps = []
    plate_axes = []
    for spec in plate_specs:
        brep, axis = _build_plate(spec)
        if brep is not None:
            plate_breps.append(brep)
        if axis is not None:
            plate_axes.append(axis)

    full_plate_breps = []
    if rg is not None and ref_full_plate_breps:
        full_plate_breps = _transform_full_plate_breps(
            ref_full_plate_breps,
            source_full_specs,
            full_plate_specs,
        )
    if not full_plate_breps:
        for _fp_idx, _fp_spec in enumerate(full_plate_specs):
            try:
                _fp_brep = _build_full_plate(_fp_spec)
                if _fp_brep is not None:
                    full_plate_breps.append(_fp_brep)
                else:
                    print(
                        "[BPG] full_plate_brep[{0}] ToBrep returned None "
                        "(az={1:.1f} alt={2:.1f} L={3:.4f} W={4:.4f} T={5:.4f})".format(
                            _fp_idx,
                            _fp_spec.get("azimuth_deg", "?"),
                            _fp_spec.get("inclination_deg", "?"),
                            _fp_spec.get("full_length", _fp_spec.get("length")) or 0,
                            _fp_spec.get("width") or 0,
                            _fp_spec.get("thickness") or 0,
                        )
                    )
            except Exception as _fp_exc:
                print("[BPG] full_plate_brep[{0}] build exception: {1}".format(_fp_idx, _fp_exc))
        print("[BPG] full_plate_breps built: {0}/{1}".format(len(full_plate_breps), len(full_plate_specs)))

    plate_hole_centers_built, plate_hole_curves, plate_hole_cutters, _per_plate_hole_centers = _plate_hole_geometry(
        active_plate_hole_specs,
        plate_specs,
    )
    plate_hole_center_counts = [len(centers) for centers in _per_plate_hole_centers]
    plate_hole_pattern_diagnostics = _plate_hole_pattern_diagnostics(
        active_plate_hole_specs,
        plate_specs,
        _per_plate_hole_centers,
    )
    for diagnostic in plate_hole_pattern_diagnostics:
        if diagnostic.get("status") != "OK":
            plate_hole_warnings.append(
                "Plate {0}: hole pattern status {1}; expected={2}, generated={3}, inside_plate={4}.".format(
                    diagnostic.get("plate_index"),
                    diagnostic.get("status"),
                    diagnostic.get("expected_count"),
                    diagnostic.get("generated_count"),
                    diagnostic.get("inside_plate_count"),
                )
            )
    plate_body_hole_messages = []
    plate_body_hole_diagnostics = []
    if plate_breps:
        plate_breps, plate_body_hole_messages, plate_body_hole_diagnostics = _cut_plate_holes_from_breps(
            plate_breps,
            active_plate_hole_specs,
            plate_specs,
        )
    repattern_messages = []
    repattern_needed = (
        plate_hole_override_connected
        or plate_body_scaled
        or plate_hole_pattern_changed
        or not ref_full_plate_breps
    )
    if full_plate_breps and repattern_needed:
        full_plate_breps, repattern_messages = _repattern_full_plate_breps(
            full_plate_breps,
            transformed_source_plate_hole_specs,
            full_plate_hole_specs,
            full_plate_specs,
        )

    stiffener_breps = []
    stiffener_specs = []
    if include_stiffeners:
        baseplate_top_plane = _translated_plane_world_z(baseplate_plane, base_thickness)
        stiffener_specs = _stiffener_specs_from_inputs(
            plane,
            target_plate_specs=plate_specs,
            baseplate_top_plane=baseplate_top_plane,
            plate_timber_widths=plate_timber_widths,
            plate_timber_heights=plate_timber_heights,
            bottom_face_mode=bottom_face_mode,
            ref_with_stiffeners_brep=ref_with_stiffeners_brep,
            ref_stiffeners=ref_stiffeners,
            stiffener_centers=stiffener_centers,
            stiffener_azimuths=stiffener_azimuths,
            stiffener_lengths=stiffener_lengths,
            stiffener_widths=stiffener_widths,
            stiffener_heights=stiffener_heights,
            stiffener_low_heights=stiffener_low_heights,
            stiffener_thicknesses=stiffener_thicknesses,
            stiffener_pair_axis_shift=stiffener_pair_axis_shift,
            stiffener_pair_from_point=stiffener_pair_from_point,
            stiffener_pair_to_point=stiffener_pair_to_point,
        )
        use_legacy_stiffener_breps = bool(
            rg is not None
            and ref_stiffeners
            and not any(spec.get("target_source") == "web_plate_targets" for spec in stiffener_specs)
        )
        if use_legacy_stiffener_breps:
            source_stiffener_specs = extract_oriented_brep_specs(
                ref_stiffeners,
                target_width=43.301,
                target_thickness=12.5,
            )
            stiffener_breps = _transform_stiffener_breps(
                ref_stiffeners,
                source_stiffener_specs,
                stiffener_specs,
            )
        if not stiffener_breps:
            stiffener_breps = [
                brep for brep in (_build_stiffener(spec) for spec in stiffener_specs)
                if brep is not None
            ]

    active_web_plate_breps = [brep for brep in plate_breps if brep is not None]
    plate_build_modes = [spec.get("plate_build_mode") for spec in plate_specs]
    if active_web_plate_breps and all(mode == "filleted_profile" for mode in plate_build_modes):
        active_web_plate_source = "filleted_profile_plate_breps"
    elif active_web_plate_breps and any(mode == "filleted_profile" for mode in plate_build_modes):
        active_web_plate_source = "mixed_profile_plate_breps"
    else:
        active_web_plate_source = "box_plate_breps"
    print("[BPG] active_web_plate_breps: {0} ({1})".format(
        len(active_web_plate_breps),
        active_web_plate_source,
    ))
    merged_footing_breps, merge_messages, web_plate_trim_succeeded = _trim_web_plates_by_baseplate(
        base_plate,
        active_web_plate_breps,
    )
    if web_plate_trim_succeeded and len(merged_footing_breps) > 1:
        plate_breps = list(merged_footing_breps[1:])
    active_base_plate = (
        merged_footing_breps[0]
        if web_plate_trim_succeeded and merged_footing_breps
        else base_plate
    )
    verification = _verify_breps(active_base_plate, full_plate_breps, stiffener_breps)
    verification["plate_hole_pattern_overridden"] = plate_hole_override_connected
    verification["plate_hole_sizing_applied"] = plate_hole_sizing_applied
    verification["plate_hole_pattern_changed"] = plate_hole_pattern_changed
    verification["plate_body_scaled"] = plate_body_scaled
    verification["plate_body_hole_cut_attempted"] = bool(plate_breps)
    verification["plate_body_hole_diagnostics"] = plate_body_hole_diagnostics
    verification["plate_hole_repatterned"] = bool(repattern_needed and full_plate_breps)
    verification["plate_hole_cutter_count"] = len(plate_hole_cutters)
    verification["active_web_plate_count"] = len(active_web_plate_breps)
    verification["active_web_plate_source"] = active_web_plate_source
    verification["base_web_difference_succeeded"] = web_plate_trim_succeeded
    verification["web_plate_trim_succeeded"] = web_plate_trim_succeeded
    verification["merged_footing_count"] = len(merged_footing_breps)
    verification["messages"].extend(plate_hole_warnings)
    verification["messages"].extend(plate_body_hole_messages)
    verification["messages"].extend(repattern_messages)
    verification["messages"].extend(merge_messages)
    metadata = _build_metadata_handoff(
        base_length,
        base_width,
        base_thickness,
        hole_diameter,
        hole_specs,
        plate_specs,
        full_plate_specs,
        active_plate_hole_specs,
        _per_plate_hole_centers,
        plate_hole_center_counts,
        plate_hole_pattern_diagnostics,
        stiffener_specs,
        verification,
        bottom_face_mode=bottom_face_mode,
        baseplate_top_z=resolved_baseplate_top_z,
        baseplate_top_source=baseplate_top_source,
        baseplate_bottom_offset_z=baseplate_bottom_offset_z,
        plate_support_heel_z_values=plate_support_heel_z_values,
        plate_support_overlap=FOOTING_BOOLEAN_OVERLAP,
        base_shape=normalized_base_shape,
        base_diameter=base_diameter_value,
        heel_fillet_radius=resolved_heel_fillet_radius,
        timber_bottom_gap=resolved_timber_bottom_gap,
        min_timber_gap=resolved_min_timber_gap,
        sizing_source=sizing_source,
        applied_sizing_recommendations=applied_sizing_recommendations,
    )

    params = {
        "source_note": SOURCE_NOTE,
        "units": SOURCE_UNITS,
        "base_length": base_length,
        "base_width": base_width,
        "base_shape": normalized_base_shape,
        "base_diameter": base_diameter_value,
        "base_thickness": base_thickness,
        "baseplate_top_z": resolved_baseplate_top_z,
        "baseplate_top_source": baseplate_top_source,
        "baseplate_bottom_offset_z": baseplate_bottom_offset_z,
        "plate_support_heel_points": plate_support_heel_points,
        "plate_support_heel_z_values": plate_support_heel_z_values,
        "plate_support_overlap": FOOTING_BOOLEAN_OVERLAP,
        "heel_fillet_radius": resolved_heel_fillet_radius,
        "timber_bottom_gap": resolved_timber_bottom_gap,
        "min_timber_gap": resolved_min_timber_gap,
        "edge_spacing": edge_spacing_tuple,
        "hole_spacing": hole_spacing_pair,
        "hole_diameter": hole_diameter,
        "hole_centers": centers,
        "plate_specs": plate_specs,
        "full_plate_specs": full_plate_specs,
        "plate_hole_specs": active_plate_hole_specs,
        "plate_hole_center_counts": plate_hole_center_counts,
        "plate_hole_pattern_diagnostics": plate_hole_pattern_diagnostics,
        "full_plate_hole_specs": full_plate_hole_specs,
        "plate_hole_centers": plate_hole_centers_built,
        "active_web_plate_source": active_web_plate_source,
        "sizing_source": sizing_source,
        "applied_sizing_recommendations": applied_sizing_recommendations,
        "timber_element_count": len(_flatten_values(timber_elements)),
        "include_stiffeners": include_stiffeners,
        "stiffener_specs": stiffener_specs,
        "stiffener_count": len(stiffener_breps),
        "merged_footing_count": len(merged_footing_breps),
        "verification": verification,
        "metadata": metadata,
    }

    return {
        "summary_text": _summarize(
            plate_specs,
            full_plate_specs,
            hole_specs,
            active_plate_hole_specs,
            include_stiffeners,
            base_length,
            base_width,
            verification,
        ),
        "base_plate": active_base_plate,
        "base_outline": base_outline,
        "plate_breps": plate_breps,
        "full_plate_breps": full_plate_breps,
        "plate_axes": plate_axes,
        "hole_curves": hole_curves,
        "hole_centers": centers,
        "hole_specs": hole_specs,
        "plate_hole_curves": plate_hole_curves,
        "plate_hole_centers": plate_hole_centers_built,
        "plate_hole_specs": active_plate_hole_specs,
        "full_plate_hole_specs": full_plate_hole_specs,
        "plate_hole_cutters": plate_hole_cutters,
        "stiffener_breps": stiffener_breps,
        "merged_footing_breps": merged_footing_breps,
        "verification": verification,
        "metadata": metadata,
        "params": params,
    }

