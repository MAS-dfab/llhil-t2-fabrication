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
        data_dir / "meters_shifted_lines.json",
        data_dir / "0806_shifted_lines.json",
        data_dir / "shifted_lines.json",
    ]
    candidates: List[Path] = [p for p in preferred if p.exists()]
    if data_dir.exists():
        for p in sorted(data_dir.glob("*shifted_lines*.json")):
            if p not in candidates:
                candidates.append(p)
    return candidates


def resolve_latest_line_model_path(project_root: Optional[Path] = None) -> Path:
    root = project_root or Path(__file__).resolve().parents[3]
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
            height = float(attrs.get("height") or 100.0)
            group_raw = attrs.get("group")
            group = int(group_raw) if group_raw is not None else None
            level_raw = attrs.get("level")
            level = int(level_raw) if level_raw is not None else None

            member_id = "{0}-{1}".format(u_key, v_key)
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
    mode = (bottom_face_mode or "Perpendicular_to_grain").strip()
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
    plate_length: float = 800.0,
    plate_width: float = 800.0,
    plate_thickness: float = 20.0,
    bottom_face_mode: str = "Perpendicular_to_grain",
) -> List[BasePlateRecord]:
    records: List[BasePlateRecord] = []
    for member in members:
        center = _base_end_point(member)
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
) -> Dict[str, object]:
    grouped: Dict[str, int] = {}
    for m in members:
        key = str(m.group) if m.group is not None else "None"
        grouped[key] = grouped.get(key, 0) + 1

    return {
        "metadata": {
            "module": "base_plate_geometry",
            "line_model_path": str(line_model_path),
            "generated_utc": datetime.utcnow().isoformat() + "Z",
            "member_count": len(members),
            "group_counts": grouped,
            "bottom_face_mode": bottom_face_mode,
        },
        "members": [asdict(m) for m in members],
        "base_plates": [asdict(p) for p in base_plates],
    }


def build_geometry_payload(
    line_model_path: Optional[Path] = None,
    plate_length: float = 800.0,
    plate_width: float = 800.0,
    plate_thickness: float = 20.0,
    bottom_face_mode: str = "Perpendicular_to_grain",
    include_hierarchies: Optional[Sequence[str]] = None,
) -> Dict[str, object]:
    path = line_model_path or resolve_latest_line_model_path()
    members = load_member_records(path)

    if include_hierarchies:
        wanted = set(include_hierarchies)
        members = [m for m in members if m.hierarchy in wanted]

    base_plates = build_base_plate_records(
        members=members,
        plate_length=plate_length,
        plate_width=plate_width,
        plate_thickness=plate_thickness,
        bottom_face_mode=bottom_face_mode,
    )
    return to_payload_dict(members, base_plates, path, bottom_face_mode)


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
        breps.append(box.ToBrep())
    return breps


def _cli() -> None:
    payload = build_geometry_payload()
    print(json.dumps(payload["metadata"], indent=2))


if __name__ == "__main__":
    _cli()
