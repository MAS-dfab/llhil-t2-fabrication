"""
Grasshopper-facing helper for post-processing base-footing dimensions and labels.

The helper is intentionally metadata-led: geometry establishes where annotations
sit, while the metadata handoff says what each annotation should report.
"""

import math

try:
    import scriptcontext as sc
except Exception:
    sc = None

try:
    import Rhino.Geometry as rg
except Exception:
    rg = None


SECTION_KEYS = ("geometry", "milling", "forces", "checks", "installation", "references")

GEOMETRY_SOURCE_OPTIONS = {
    "auto": 0,
    "package": 1,
    "manual": 2,
}

# GH scripts can reference these names directly; provide safe defaults when
# the component has differently named or disconnected inputs.
if "package" not in globals():
    package = None
if "payload" not in globals():
    payload = None


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


def _as_dict(value):
    if isinstance(value, dict):
        return dict(value)
    keys = getattr(value, "Keys", None)
    if keys is not None:
        try:
            return {key: value[key] for key in list(keys)}
        except Exception:
            pass
    return {}


def _to_python_data(value):
    if isinstance(value, dict):
        return {key: _to_python_data(item) for key, item in value.items()}
    keys = getattr(value, "Keys", None)
    if keys is not None:
        try:
            return {
                _to_python_data(key): _to_python_data(value[key])
                for key in list(keys)
            }
        except Exception:
            pass
    if isinstance(value, (list, tuple)):
        return [_to_python_data(item) for item in value]
    if not isinstance(value, (str, bytes)):
        try:
            value.Count
            return [_to_python_data(item) for item in value]
        except Exception:
            pass
    return value


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


def _iter_dicts_deep(value, _seen=None):
    if _seen is None:
        _seen = set()
    marker = id(value)
    if marker in _seen:
        return
    _seen.add(marker)

    if isinstance(value, dict):
        yield value
        for child in value.values():
            for nested in _iter_dicts_deep(child, _seen):
                yield nested
        return

    if isinstance(value, (list, tuple)):
        for item in value:
            for nested in _iter_dicts_deep(item, _seen):
                yield nested


def _first_non_empty_lists(candidates):
    for candidate in candidates:
        values = _flatten_values(candidate)
        values = [value for value in values if value is not None]
        if values:
            return values
    return []


def _extract_package_geometry(payload):
    data = _unwrap_payload(payload)
    if not isinstance(data, dict):
        return {
            "timber_brep": [],
            "plate_brep": [],
            "bolt_points": [],
            "washer_recess_breps": [],
            "slot_cut_brep": [],
            "metadata": {},
            "messages": ["Package input did not normalize to a dictionary payload."],
            "raw": data,
        }

    dicts = list(_iter_dicts_deep(data))

    timber_keys = ("timber_brep", "timber_breps", "timber", "timber_geometry", "inspection_breps")
    plate_keys = ("plate_brep", "plate_breps", "base_plates", "preview_breps", "footing_breps")
    bolt_keys = ("bolt_points", "hole_centers", "anchor_points", "bolt_hole_centers")
    washer_keys = ("washer_recess_brep", "washer_recess_breps", "washer_recesses")
    slot_keys = ("slot_cut_brep", "slot_cut_breps", "slot_brep", "slot_breps", "slot_cuts")

    def pick_values(keys):
        for dictionary in dicts:
            values = _first_non_empty_lists([dictionary.get(key) for key in keys])
            if values:
                return values
        return []

    metadata = {}
    for dictionary in dicts:
        handoff = _as_dict(dictionary.get("handoff"))
        if handoff:
            metadata = handoff
            break
    if not metadata:
        for dictionary in dicts:
            candidate = _as_dict(dictionary.get("metadata"))
            if candidate:
                metadata = candidate
                break

    messages = []
    geometry_payload = {
        "timber_brep": pick_values(timber_keys),
        "plate_brep": pick_values(plate_keys),
        "bolt_points": pick_values(bolt_keys),
        "washer_recess_breps": pick_values(washer_keys),
        "slot_cut_brep": pick_values(slot_keys),
        "metadata": metadata,
        "messages": messages,
        "raw": data,
    }
    return geometry_payload


def _coerce_source_mode(value, default="auto"):
    if value is None:
        return default
    if isinstance(value, bool):
        return "package" if value else "manual"
    try:
        numeric = int(value)
        for name, code in GEOMETRY_SOURCE_OPTIONS.items():
            if numeric == code:
                return name
    except Exception:
        pass
    text = str(value).strip().lower().replace(" ", "_")
    if text in GEOMETRY_SOURCE_OPTIONS:
        return text
    aliases = {
        "payload": "package",
        "pkg": "package",
        "direct": "manual",
        "manual_input": "manual",
        "auto_prefer_payload": "auto",
    }
    return aliases.get(text, default)


def _pick_source_items(mode, package_items, manual_items):
    if mode == "package":
        return list(package_items or [])
    if mode == "manual":
        return list(manual_items or [])
    if package_items:
        return list(package_items)
    return list(manual_items or [])


def _metadata_value(metadata, key, default=None):
    data = _as_dict(metadata)
    if key in data and data[key] is not None:
        return data[key]
    for section in SECTION_KEYS:
        nested = _as_dict(data.get(section))
        if key in nested and nested[key] is not None:
            return nested[key]
    return default


def _point_xyz(value):
    if value is None:
        return None
    if hasattr(value, "X") and hasattr(value, "Y") and hasattr(value, "Z"):
        return (float(value.X), float(value.Y), float(value.Z))
    try:
        items = list(value)
    except Exception:
        return None
    if len(items) < 3:
        return None
    try:
        return (float(items[0]), float(items[1]), float(items[2]))
    except Exception:
        return None


def _as_point(value):
    xyz = _point_xyz(value)
    if xyz is None:
        return None
    if rg is None:
        return xyz
    return rg.Point3d(*xyz)


def _geometry_bbox(value):
    if rg is None or value is None:
        return None
    getter = getattr(value, "GetBoundingBox", None)
    if callable(getter):
        try:
            return getter(True)
        except Exception:
            return None
    return None


def _combined_bbox(items):
    if rg is None:
        return None
    bbox = rg.BoundingBox.Empty
    found = False
    for item in items or []:
        geometry_bbox = _geometry_bbox(item)
        if geometry_bbox is not None and geometry_bbox.IsValid:
            bbox.Union(geometry_bbox)
            found = True
    return bbox if found else None


def _bbox_corners(bbox):
    if rg is None or bbox is None or not bbox.IsValid:
        return []
    return list(bbox.GetCorners())


def _bbox_center(bbox):
    if rg is None or bbox is None or not bbox.IsValid:
        return None
    return bbox.Center


def _bbox_from_points(points):
    if rg is None:
        return None
    valid = [point for point in points or [] if point is not None]
    if not valid:
        return None
    return rg.BoundingBox(valid)


def _merge_bboxes(*boxes):
    if rg is None:
        return None
    merged = rg.BoundingBox.Empty
    found = False
    for box in boxes:
        if box is not None and box.IsValid:
            merged.Union(box)
            found = True
    return merged if found else None


def _view_planes(center):
    if rg is None or center is None:
        return {
            "top": None,
            "front": None,
            "section": None,
        }
    return {
        "top": rg.Plane(center, rg.Vector3d.XAxis, rg.Vector3d.YAxis),
        "front": rg.Plane(center, rg.Vector3d.XAxis, rg.Vector3d.ZAxis),
        "section": rg.Plane(center, rg.Vector3d.YAxis, rg.Vector3d.ZAxis),
    }


def _local_point(plane, point):
    if rg is None or plane is None or point is None:
        return None
    vector = point - plane.Origin
    return (
        vector.X * plane.XAxis.X + vector.Y * plane.XAxis.Y + vector.Z * plane.XAxis.Z,
        vector.X * plane.YAxis.X + vector.Y * plane.YAxis.Y + vector.Z * plane.YAxis.Z,
    )


def _extents_on_plane(bbox, plane):
    local = [_local_point(plane, point) for point in _bbox_corners(bbox)]
    local = [item for item in local if item is not None]
    if not local:
        return None
    return (
        min(item[0] for item in local),
        max(item[0] for item in local),
        min(item[1] for item in local),
        max(item[1] for item in local),
    )


def _point_on_plane(plane, x, y):
    if rg is None or plane is None:
        return (x, y, 0.0)
    point = rg.Point3d(plane.Origin)
    point += plane.XAxis * float(x)
    point += plane.YAxis * float(y)
    return point


def _line(start, end):
    if rg is None:
        return {"type": "line", "start": start, "end": end}
    return rg.Line(start, end).ToNurbsCurve()


def _text(text, plane, height):
    if rg is None or plane is None:
        return {"type": "text", "text": str(text), "plane": plane, "height": height}
    try:
        entity = rg.TextEntity()
        entity.Text = str(text)
        entity.Plane = plane
        entity.TextHeight = float(height)
        return entity
    except Exception:
        return {"type": "text", "text": str(text), "plane": plane, "height": height}


def _shifted_text_plane(plane, point):
    if rg is None or plane is None or point is None:
        return None
    return rg.Plane(point, plane.XAxis, plane.YAxis)


def _vector_length_2d(dx, dy):
    return math.sqrt(dx * dx + dy * dy)


def _dimension(plane, start_xy, end_xy, offset, label, text_height, arrow_size):
    x0, y0 = start_xy
    x1, y1 = end_xy
    dx = x1 - x0
    dy = y1 - y0
    length = _vector_length_2d(dx, dy)
    if length <= 1e-9:
        return None

    tx = dx / length
    ty = dy / length
    nx = -ty
    ny = tx
    dim_start_xy = (x0 + nx * offset, y0 + ny * offset)
    dim_end_xy = (x1 + nx * offset, y1 + ny * offset)

    start = _point_on_plane(plane, x0, y0)
    end = _point_on_plane(plane, x1, y1)
    dim_start = _point_on_plane(plane, dim_start_xy[0], dim_start_xy[1])
    dim_end = _point_on_plane(plane, dim_end_xy[0], dim_end_xy[1])
    text_point = _point_on_plane(
        plane,
        0.5 * (dim_start_xy[0] + dim_end_xy[0]),
        0.5 * (dim_start_xy[1] + dim_end_xy[1]),
    )

    angle = math.radians(25.0)
    ax = arrow_size * math.cos(angle)
    ay = arrow_size * math.sin(angle)
    arrow_start_a = _point_on_plane(plane, dim_start_xy[0] + tx * ax + nx * ay, dim_start_xy[1] + ty * ax + ny * ay)
    arrow_start_b = _point_on_plane(plane, dim_start_xy[0] + tx * ax - nx * ay, dim_start_xy[1] + ty * ax - ny * ay)
    arrow_end_a = _point_on_plane(plane, dim_end_xy[0] - tx * ax + nx * ay, dim_end_xy[1] - ty * ax + ny * ay)
    arrow_end_b = _point_on_plane(plane, dim_end_xy[0] - tx * ax - nx * ay, dim_end_xy[1] - ty * ax - ny * ay)

    return {
        "dim_line": _line(dim_start, dim_end),
        "extension_lines": [_line(start, dim_start), _line(end, dim_end)],
        "arrowheads": [
            _line(dim_start, arrow_start_a),
            _line(dim_start, arrow_start_b),
            _line(dim_end, arrow_end_a),
            _line(dim_end, arrow_end_b),
        ],
        "text": _text(label, _shifted_text_plane(plane, text_point), text_height),
    }


def _leader_label(plane, anchor_xy, landing_xy, text, text_height):
    anchor = _point_on_plane(plane, anchor_xy[0], anchor_xy[1])
    landing = _point_on_plane(plane, landing_xy[0], landing_xy[1])
    return {
        "leader": _line(anchor, landing),
        "text": _text(text, _shifted_text_plane(plane, landing), text_height),
    }


def _fmt(label, value, suffix=""):
    numeric = _coerce_float(value, None)
    if numeric is None:
        return label
    return "{0} {1:.1f}{2}".format(label, numeric, suffix)


def _unique_sorted(values, tolerance=1e-6):
    result = []
    for value in sorted(values):
        if not result or abs(value - result[-1]) > tolerance:
            result.append(value)
    return result


def _project_points(points, plane):
    local = []
    for point in points or []:
        next_point = _local_point(plane, point)
        if next_point is not None:
            local.append(next_point)
    return local


def _append_dimension(targets, group, payload):
    if not payload:
        return
    targets["dim_lines"].append(payload["dim_line"])
    targets["extension_lines"].extend(payload["extension_lines"])
    targets["arrowheads"].extend(payload["arrowheads"])
    targets["text_labels"].append(payload["text"])
    targets["annotation_groups"][group].append(payload)


def _append_leader(targets, group, payload):
    if not payload:
        return
    targets["leader_lines"].append(payload["leader"])
    targets["text_labels"].append(payload["text"])
    targets["annotation_groups"][group].append(payload)


def _build_check_labels(metadata, plane, extents, text_height):
    checks = _as_dict(_as_dict(metadata).get("checks"))
    labels = []
    source = [
        ("bolt_shear_utilization", "Bolt shear"),
        ("timber_embedment_utilization", "Timber embedment"),
        ("washer_bearing_utilization", "Washer bearing"),
        ("plate_net_section_utilization", "Plate net section"),
        ("weld_check_status", "Weld check"),
        ("anchor_screw_pier_check_status", "Anchor / screw-pier"),
    ]
    x0, x1, y0, y1 = extents
    cursor_y = y1 + text_height * 2.0
    for key, title in source:
        value = checks.get(key)
        if value is None:
            value = _metadata_value(metadata, key, None)
        if value is None:
            continue
        if isinstance(value, (int, float)):
            text = "{0}: {1:.2f}".format(title, float(value))
        else:
            text = "{0}: {1}".format(title, value)
        point = _point_on_plane(plane, x0, cursor_y)
        labels.append(_text(text, _shifted_text_plane(plane, point), text_height))
        cursor_y += text_height * 1.6
    return labels


def run(
    package=None,
    payload=None,
    timber_brep=None,
    plate_brep=None,
    bolt_points=None,
    washer_recess_breps=None,
    slot_cut_brep=None,
    timber_source="auto",
    plate_source="auto",
    bolt_source="auto",
    washer_source="auto",
    slot_source="auto",
    metadata=None,
    show_top_dimensions=True,
    show_front_dimensions=True,
    show_section_dimensions=True,
    show_code_labels=True,
    dim_offset=None,
    text_height=None,
    arrow_size=None,
    view_scale=1.0,
):
    package_input = package if package is not None else payload
    parsed = _extract_package_geometry(package_input) if package_input is not None else {
        "timber_brep": [],
        "plate_brep": [],
        "bolt_points": [],
        "washer_recess_breps": [],
        "slot_cut_brep": [],
        "metadata": {},
        "messages": [],
        "raw": None,
    }

    timber_mode = _coerce_source_mode(timber_source, "auto")
    plate_mode = _coerce_source_mode(plate_source, "auto")
    bolt_mode = _coerce_source_mode(bolt_source, "auto")
    washer_mode = _coerce_source_mode(washer_source, "auto")
    slot_mode = _coerce_source_mode(slot_source, "auto")

    manual_timber = _flatten_values(timber_brep)
    manual_plate = _flatten_values(plate_brep)
    manual_washer = _flatten_values(washer_recess_breps)
    manual_slot = _flatten_values(slot_cut_brep)
    manual_bolts = _flatten_values(bolt_points)

    timber_items = _pick_source_items(timber_mode, parsed["timber_brep"], manual_timber)
    plate_items = _pick_source_items(plate_mode, parsed["plate_brep"], manual_plate)
    washer_items = _pick_source_items(washer_mode, parsed["washer_recess_breps"], manual_washer)
    slot_items = _pick_source_items(slot_mode, parsed["slot_cut_brep"], manual_slot)
    bolt_values = _pick_source_items(bolt_mode, parsed["bolt_points"], manual_bolts)

    metadata = metadata if metadata is not None else parsed["metadata"]
    bolts = [_as_point(value) for value in bolt_values]
    bolts = [point for point in bolts if point is not None]

    scale = max(_coerce_float(view_scale, 1.0), 1e-9)
    dim_offset = _coerce_float(dim_offset, 25.0) / scale
    text_height = _coerce_float(text_height, 8.0) / scale
    arrow_size = _coerce_float(arrow_size, 3.0) / scale

    timber_bbox = _combined_bbox(timber_items)
    plate_bbox = _combined_bbox(plate_items)
    washer_bbox = _combined_bbox(washer_items)
    slot_bbox = _combined_bbox(slot_items)
    bolt_bbox = _bbox_from_points(bolts)
    overall_bbox = _merge_bboxes(timber_bbox, plate_bbox, washer_bbox, slot_bbox, bolt_bbox)
    center = _bbox_center(overall_bbox)
    planes = _view_planes(center)

    outputs = {
        "dim_lines": [],
        "extension_lines": [],
        "arrowheads": [],
        "text_labels": [],
        "leader_lines": [],
        "code_check_labels": [],
        "annotation_groups": {
            "top": [],
            "front": [],
            "section": [],
            "code": [],
        },
        "metadata": metadata or {},
        "annotation_planes": planes,
        "source_report": {
            "timber_source": timber_mode,
            "plate_source": plate_mode,
            "bolt_source": bolt_mode,
            "washer_source": washer_mode,
            "slot_source": slot_mode,
            "timber_count": len(timber_items),
            "plate_count": len(plate_items),
            "bolt_count": len(bolts),
            "washer_count": len(washer_items),
            "slot_count": len(slot_items),
            "has_package": bool(package_input is not None),
        },
    }
    outputs["messages"] = list(parsed.get("messages") or [])
    if overall_bbox is None or center is None:
        outputs["messages"].append("No valid geometry or bolt points were supplied for annotation extents.")
        outputs["rh_geo_preview"] = []
        outputs["reports"] = {
            "dim_lines": outputs["dim_lines"],
            "extension_lines": outputs["extension_lines"],
            "arrowheads": outputs["arrowheads"],
            "text_labels": outputs["text_labels"],
            "leader_lines": outputs["leader_lines"],
            "annotation_groups": outputs["annotation_groups"],
            "annotation_planes": outputs["annotation_planes"],
            "metadata": outputs["metadata"],
            "source_report": outputs["source_report"],
        }
        return outputs

    top_extents = _extents_on_plane(overall_bbox, planes["top"])
    front_extents = _extents_on_plane(overall_bbox, planes["front"])
    section_extents = _extents_on_plane(overall_bbox, planes["section"])

    if _coerce_bool(show_top_dimensions, True):
        plate_top = _extents_on_plane(plate_bbox or overall_bbox, planes["top"])
        timber_top = _extents_on_plane(timber_bbox or overall_bbox, planes["top"])
        slot_top = _extents_on_plane(slot_bbox, planes["top"]) if slot_bbox else None
        bolt_top = _project_points(bolts, planes["top"])

        if plate_top:
            _append_dimension(
                outputs,
                "top",
                _dimension(
                    planes["top"],
                    (plate_top[0], plate_top[2]),
                    (plate_top[1], plate_top[2]),
                    -dim_offset,
                    _fmt("plate L", _metadata_value(metadata, "plate_length")),
                    text_height,
                    arrow_size,
                ),
            )
        if slot_top:
            _append_dimension(
                outputs,
                "top",
                _dimension(
                    planes["top"],
                    (slot_top[0], slot_top[3]),
                    (slot_top[1], slot_top[3]),
                    dim_offset,
                    _fmt("slot L", _metadata_value(metadata, "slot_length")),
                    text_height,
                    arrow_size,
                ),
            )
        if timber_top:
            _append_dimension(
                outputs,
                "top",
                _dimension(
                    planes["top"],
                    (timber_top[0], timber_top[2]),
                    (timber_top[0], timber_top[3]),
                    -dim_offset,
                    _fmt("timber W", _metadata_value(metadata, "timber_width")),
                    text_height,
                    arrow_size,
                ),
            )
        if plate_top and bolt_top:
            x_values = _unique_sorted([item[0] for item in bolt_top])
            if len(x_values) >= 2:
                for index in range(len(x_values) - 1):
                    _append_dimension(
                        outputs,
                        "top",
                        _dimension(
                            planes["top"],
                            (x_values[index], plate_top[3]),
                            (x_values[index + 1], plate_top[3]),
                            dim_offset,
                            _fmt("pitch", _metadata_value(metadata, "pitch_parallel")),
                            text_height,
                            arrow_size,
                        ),
                    )
                _append_dimension(
                    outputs,
                    "top",
                    _dimension(
                        planes["top"],
                        (plate_top[0], plate_top[2]),
                        (x_values[0], plate_top[2]),
                        -2.0 * dim_offset,
                        _fmt("end", _metadata_value(metadata, "end_distance")),
                        text_height,
                        arrow_size,
                    ),
                )
                _append_dimension(
                    outputs,
                    "top",
                    _dimension(
                        planes["top"],
                        (x_values[-1], plate_top[2]),
                        (plate_top[1], plate_top[2]),
                        -2.0 * dim_offset,
                        _fmt("end", _metadata_value(metadata, "end_distance")),
                        text_height,
                        arrow_size,
                    ),
                )
        if washer_bbox:
            washer_top = _extents_on_plane(washer_bbox, planes["top"])
            washer_center = _local_point(planes["top"], washer_bbox.Center)
            if washer_top and washer_center:
                _append_leader(
                    outputs,
                    "top",
                    _leader_label(
                        planes["top"],
                        washer_center,
                        (washer_top[1] + dim_offset, washer_top[3] + dim_offset),
                        _fmt("washer pocket dia", _metadata_value(metadata, "washer_face_diameter")),
                        text_height,
                    ),
                )

    if _coerce_bool(show_front_dimensions, True):
        plate_front = _extents_on_plane(plate_bbox or overall_bbox, planes["front"])
        timber_front = _extents_on_plane(timber_bbox or overall_bbox, planes["front"])
        bolt_front = _project_points(bolts, planes["front"])
        if plate_front:
            _append_dimension(
                outputs,
                "front",
                _dimension(
                    planes["front"],
                    (plate_front[0], plate_front[2]),
                    (plate_front[0], plate_front[3]),
                    -dim_offset,
                    _fmt("plate D", _metadata_value(metadata, "plate_depth")),
                    text_height,
                    arrow_size,
                ),
            )
        if timber_front:
            _append_dimension(
                outputs,
                "front",
                _dimension(
                    planes["front"],
                    (timber_front[1], timber_front[2]),
                    (timber_front[1], timber_front[3]),
                    dim_offset,
                    _fmt("timber D", _metadata_value(metadata, "timber_depth")),
                    text_height,
                    arrow_size,
                ),
            )
        if plate_front and bolt_front:
            y_values = _unique_sorted([item[1] for item in bolt_front])
            if len(y_values) >= 2:
                _append_dimension(
                    outputs,
                    "front",
                    _dimension(
                        planes["front"],
                        (plate_front[1], y_values[0]),
                        (plate_front[1], y_values[-1]),
                        dim_offset,
                        _fmt("gage", _metadata_value(metadata, "gage_perp")),
                        text_height,
                        arrow_size,
                    ),
                )
                _append_dimension(
                    outputs,
                    "front",
                    _dimension(
                        planes["front"],
                        (plate_front[0], plate_front[2]),
                        (plate_front[0], y_values[0]),
                        -2.0 * dim_offset,
                        _fmt("edge", _metadata_value(metadata, "edge_distance")),
                        text_height,
                        arrow_size,
                    ),
                )
                _append_dimension(
                    outputs,
                    "front",
                    _dimension(
                        planes["front"],
                        (plate_front[0], y_values[-1]),
                        (plate_front[0], plate_front[3]),
                        -2.0 * dim_offset,
                        _fmt("edge", _metadata_value(metadata, "edge_distance")),
                        text_height,
                        arrow_size,
                    ),
                )
                for index, row_y in enumerate(y_values, start=1):
                    _append_leader(
                        outputs,
                        "front",
                        _leader_label(
                            planes["front"],
                            (plate_front[1], row_y),
                            (plate_front[1] + dim_offset, row_y + dim_offset),
                            "row {0}".format(index),
                            text_height,
                        ),
                    )

    if _coerce_bool(show_section_dimensions, True):
        timber_section = _extents_on_plane(timber_bbox or overall_bbox, planes["section"])
        plate_section = _extents_on_plane(plate_bbox or overall_bbox, planes["section"])
        slot_section = _extents_on_plane(slot_bbox, planes["section"]) if slot_bbox else None
        washer_section = _extents_on_plane(washer_bbox, planes["section"]) if washer_bbox else None

        if timber_section:
            _append_dimension(
                outputs,
                "section",
                _dimension(
                    planes["section"],
                    (timber_section[0], timber_section[2]),
                    (timber_section[1], timber_section[2]),
                    -dim_offset,
                    _fmt("timber W", _metadata_value(metadata, "timber_width")),
                    text_height,
                    arrow_size,
                ),
            )
        if plate_section:
            _append_dimension(
                outputs,
                "section",
                _dimension(
                    planes["section"],
                    (plate_section[0], plate_section[3]),
                    (plate_section[1], plate_section[3]),
                    dim_offset,
                    _fmt("plate T", _metadata_value(metadata, "plate_thickness")),
                    text_height,
                    arrow_size,
                ),
            )
        if slot_section:
            _append_dimension(
                outputs,
                "section",
                _dimension(
                    planes["section"],
                    (slot_section[0], slot_section[3]),
                    (slot_section[1], slot_section[3]),
                    2.0 * dim_offset,
                    _fmt("slot W", _metadata_value(metadata, "slot_width")),
                    text_height,
                    arrow_size,
                ),
            )
            if timber_section:
                _append_dimension(
                    outputs,
                    "section",
                    _dimension(
                        planes["section"],
                        (timber_section[0], timber_section[2]),
                        (slot_section[0], timber_section[2]),
                        -2.0 * dim_offset,
                        "side timber",
                        text_height,
                        arrow_size,
                    ),
                )
                _append_dimension(
                    outputs,
                    "section",
                    _dimension(
                        planes["section"],
                        (slot_section[1], timber_section[2]),
                        (timber_section[1], timber_section[2]),
                        -2.0 * dim_offset,
                        "side timber",
                        text_height,
                        arrow_size,
                    ),
                )
        if washer_section:
            _append_dimension(
                outputs,
                "section",
                _dimension(
                    planes["section"],
                    (washer_section[0], washer_section[2]),
                    (washer_section[0], washer_section[3]),
                    -dim_offset,
                    _fmt("recess D", _metadata_value(metadata, "washer_recess_depth")),
                    text_height,
                    arrow_size,
                ),
            )
            remaining = _coerce_float(_metadata_value(metadata, "timber_width"), None)
            recess = _coerce_float(_metadata_value(metadata, "washer_recess_depth"), None)
            if remaining is not None and recess is not None:
                remaining -= recess
            _append_leader(
                outputs,
                "section",
                _leader_label(
                    planes["section"],
                    (washer_section[1], washer_section[3]),
                    (washer_section[1] + dim_offset, washer_section[3] + dim_offset),
                    _fmt("remaining timber", remaining),
                    text_height,
                ),
            )

    if _coerce_bool(show_code_labels, True) and top_extents:
        outputs["code_check_labels"] = _build_check_labels(metadata, planes["top"], top_extents, text_height)
        outputs["annotation_groups"]["code"].extend(outputs["code_check_labels"])

    outputs["rh_geo_preview"] = (
        list(outputs["dim_lines"])
        + list(outputs["extension_lines"])
        + list(outputs["arrowheads"])
        + list(outputs["text_labels"])
        + list(outputs["leader_lines"])
    )
    outputs["reports"] = {
        "dim_lines": outputs["dim_lines"],
        "extension_lines": outputs["extension_lines"],
        "arrowheads": outputs["arrowheads"],
        "text_labels": outputs["text_labels"],
        "leader_lines": outputs["leader_lines"],
        "annotation_groups": outputs["annotation_groups"],
        "annotation_planes": outputs["annotation_planes"],
        "metadata": outputs["metadata"],
        "source_report": outputs["source_report"],
    }
    return outputs
