"""
Grasshopper-facing helper for organizing base-footing annotation output onto a
print-oriented sheet guide.
"""

from typing import List, Dict, Union, Any, Tuple, Set, Optional

try:
    import Rhino.Geometry as rg
except ImportError:
    class MockRhinoGeometry:
        class BoundingBox:
            Empty: 'MockRhinoGeometry.BoundingBox'

            def __init__(self, *args):
                if args:
                    self.IsValid = True
                    self.Center = MockRhinoGeometry.Point3d(0, 0, 0)
                    self.Diagonal = MockRhinoGeometry.Vector3d(0, 0, 0)
                else:
                    self.IsValid = False

            def Union(self, other: 'MockRhinoGeometry.BoundingBox') -> None:
                if other and other.IsValid:
                    self.IsValid = True

        class Point3d:
            def __init__(self, x: float, y: float, z: float):
                self.X = x
                self.Y = y
                self.Z = z

        class Polyline:
            def __init__(self, points: List['MockRhinoGeometry.Point3d']):
                self.points = points

            def ToNurbsCurve(self) -> Any:
                return None

        class Vector3d:
            XAxis = (1.0, 0.0, 0.0)
            YAxis = (0.0, 1.0, 0.0)

            def __init__(self, x: float, y: float, z: float):
                self.X = x
                self.Y = y
                self.Z = z

            @property
            def Length(self) -> float:
                return (self.X**2 + self.Y**2 + self.Z**2) ** 0.5

        class Plane:
            def __init__(self, origin: 'MockRhinoGeometry.Point3d', x_axis: Tuple[float, float, float], y_axis: Tuple[float, float, float]):
                self.Origin = origin
                self.XAxis = x_axis
                self.YAxis = y_axis

        class TextEntity:
            def __init__(self):
                self.Text = ""
                self.Plane: Optional['MockRhinoGeometry.Plane'] = None
                self.TextHeight = 0.0

    rg = MockRhinoGeometry()
    MockRhinoGeometry.BoundingBox.Empty = MockRhinoGeometry.BoundingBox()


SHEET_SIZES_MM = {
    "A4": (297.0, 210.0),
    "A3": (420.0, 297.0),
    "A2": (594.0, 420.0),
    "A1": (841.0, 594.0),
    "A0": (1189.0, 841.0),
}


def _flatten_values(value: Any, _seen: Optional[Set[int]] = None) -> List[Any]:
    if _seen is None:
        _seen = set()
    if id(value) in _seen:
        return []
    _seen.add(id(value))
    if isinstance(value, dict):
        items = []
        for nested in value.values():
            items.extend(_flatten_values(nested, _seen))
        return items
    elif isinstance(value, list):
        items = []
        for nested in value:
            items.extend(_flatten_values(nested, _seen))
        return items
    return [value]


def _coerce_float(value: Any, default: Optional[float] = None) -> Optional[float]:
    if value is None:
        return default
    try:
        return float(value)
    except Exception:
        return default


def _coerce_bool(value: Any, default: bool = False) -> bool:
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


def _as_dict(value: Any) -> Dict[Any, Any]:
    if isinstance(value, dict):
        return dict(value)
    keys = getattr(value, "Keys", None)
    if keys is not None:
        try:
            return {key: value[key] for key in list(keys)}
        except Exception:
            pass
    return {}


def _geometry_bbox(value: Any) -> Optional[Any]:
    if rg is None or value is None:
        return None
    getter = getattr(value, "GetBoundingBox", None)
    if callable(getter):
        try:
            return getter(True)
        except Exception:
            return None
    return None


def _combined_bbox(values: Any) -> Optional[Any]:
    if rg is None:
        return None
    bbox = rg.BoundingBox.Empty
    found = False
    for value in _flatten_values(values):
        geometry_bbox = _geometry_bbox(value)
        if geometry_bbox is not None and geometry_bbox.IsValid:
            bbox.Union(geometry_bbox)
            found = True
    return bbox if found else None


def _sheet_size(value: Union[str, Tuple[float, float]]) -> Tuple[float, float]:
    if isinstance(value, str):
        if value == "A3":
            return (420.0, 297.0)
        elif value == "A4":
            return (297.0, 210.0)
        else:
            raise ValueError(f"Unknown sheet size: {value}")
    return value


def _point(x: float, y: float, z: float = 0.0) -> Union[Tuple[float, float, float], 'MockRhinoGeometry.Point3d']:
    if rg is None:
        return (x, y, z)
    return rg.Point3d(float(x), float(y), float(z))


def _polyline(points: List[Union[Tuple[float, float, float], 'MockRhinoGeometry.Point3d']]) -> Union[Dict[str, Any], Any]:
    if rg is None:
        return {"type": "polyline", "points": list(points)}
    return rg.Polyline(points).ToNurbsCurve()


def _rectangle(x0: float, y0: float, x1: float, y1: float) -> Union[Dict[str, Any], Any]:
    return _polyline([
        _point(x0, y0),
        _point(x1, y0),
        _point(x1, y1),
        _point(x0, y1),
        _point(x0, y0),
    ])


def _text(text: str, x: float, y: float, height: float) -> Union[Dict[str, Any], Any]:
    if rg is None:
        return {"type": "text", "text": str(text), "point": (x, y, 0.0), "height": height}
    plane = rg.Plane(_point(x, y), rg.Vector3d.XAxis, rg.Vector3d.YAxis)
    entity = rg.TextEntity()
    entity.Text = text
    entity.Plane = plane
    entity.TextHeight = height
    return entity


def _bbox_center(bbox: Union['MockRhinoGeometry.BoundingBox', Any]) -> Union['MockRhinoGeometry.Point3d', None]:
    if rg is None or bbox is None or not bbox.IsValid:
        return None
    return bbox.Center


def _bbox_diagonal_length(bbox: Union['MockRhinoGeometry.BoundingBox', Any]) -> Union[float, None]:
    if rg is None or bbox is None or not bbox.IsValid:
        return None
    return bbox.Diagonal.Length


def _named_view_data(bbox: Optional['MockRhinoGeometry.BoundingBox'], requested: bool) -> Dict[str, Any]:
    if not requested or bbox is None or not bbox.IsValid:
        return {
            "requested": bool(requested),
            "enabled": False,
        }

    center = _bbox_center(bbox)
    diagonal_length = _bbox_diagonal_length(bbox)
    if center is None or diagonal_length is None:
        return {
            "requested": bool(requested),
            "enabled": False,
        }

    distance = max(diagonal_length * 2.0, 1.0)
    return {
        "requested": True,
        "enabled": True,
        "center": center,
        "distance": distance,
    }


def _report_lines(metadata: Dict[str, Any], combined_report: Optional[Union[str, Dict[str, Any], List[str]]]) -> List[str]:
    lines: List[str] = []

    if isinstance(combined_report, list):
        lines.extend([str(item) for item in combined_report if item is not None])
    elif isinstance(combined_report, str):
        lines.extend([line for line in combined_report.splitlines() if line.strip()])
    elif isinstance(combined_report, dict):
        for key in ("summary", "combined_report", "report_text"):
            value = combined_report.get(key)
            if isinstance(value, str) and value.strip():
                lines.extend([line for line in value.splitlines() if line.strip()])

    if not lines:
        for key in ("module", "generated_utc", "member_count"):
            value = metadata.get(key)
            if value is not None:
                lines.append("{0}: {1}".format(key, value))

    return lines


def run(
    geometry_objects: Optional[List[Any]] = None,
    annotation_objects: Optional[List[Any]] = None,
    metadata: Optional[Dict[str, Any]] = None,
    combined_report: Optional[Union[str, Dict[str, Any]]] = None,
    create_named_views: bool = True,
    create_layout_guides: bool = True,
    sheet_size: Union[str, Tuple[float, float]] = "A3",
    drawing_scale: float = 20.0,
    title_block_info: Optional[Dict[str, Any]] = None,
    project_name: str = "",
    detail_name: str = "",
) -> Dict[str, Any]:
    """
    Generate layout data for base footing.

    Args:
        geometry_objects: List of geometry objects.
        annotation_objects: List of annotation objects.
        metadata: Metadata dictionary.
        combined_report: Combined report as a string or dictionary.
        create_named_views: Whether to create named views.
        create_layout_guides: Whether to create layout guides.
        sheet_size: Sheet size as a string or tuple of dimensions.
        drawing_scale: Drawing scale factor.
        title_block_info: Title block information.
        project_name: Project name.
        detail_name: Detail name.

    Returns:
        Dictionary containing layout data.
    """
    metadata = metadata or {}
    title_block_info = _as_dict(title_block_info)
    geometry_bbox = _combined_bbox(geometry_objects)
    annotation_bbox = _combined_bbox(annotation_objects)
    overall_bbox = geometry_bbox or annotation_bbox
    if rg is not None and geometry_bbox is not None and annotation_bbox is not None:
        overall_bbox = rg.BoundingBox(geometry_bbox)
        overall_bbox.Union(annotation_bbox)

    sheet_width, sheet_height = _sheet_size(sheet_size)
    scale = _coerce_float(drawing_scale, 20.0)
    margin = 12.0
    title_height = 38.0
    gutter = 8.0
    usable_height = sheet_height - title_height - 2.0 * margin
    usable_width = sheet_width - 2.0 * margin
    half_width = 0.5 * (usable_width - gutter)
    half_height = 0.5 * (usable_height - gutter)

    create_layout_guides = _coerce_bool(create_layout_guides, True)
    sheet_boundary = _rectangle(0.0, 0.0, sheet_width, sheet_height) if create_layout_guides else None
    viewport_guides = []
    title_block_curves = []
    title_block_text = []
    view_labels = []
    report_text_block = []

    viewport_defs = {
        "NODE_TOP": (margin, sheet_height - margin - half_height, margin + half_width, sheet_height - margin),
        "NODE_FRONT": (margin + half_width + gutter, sheet_height - margin - half_height, sheet_width - margin, sheet_height - margin),
        "NODE_SECTION": (margin, margin + title_height, margin + half_width, margin + title_height + half_height),
        "NODE_ISO": (margin + half_width + gutter, margin + title_height, sheet_width - margin, margin + title_height + half_height),
    }

    if create_layout_guides:
        viewport_guides = [_rectangle(*coords) for coords in viewport_defs.values()]
        title_block_curves = [
            _rectangle(margin, margin, sheet_width - margin, margin + title_height),
            _rectangle(sheet_width - margin - 120.0, margin, sheet_width - margin, margin + title_height),
        ]
        for name, coords in viewport_defs.items():
            x0, y0, _x1, y1 = coords
            view_labels.append(_text("{0}  1:{1:g}".format(name, scale), x0 + 3.0, y1 - 8.0, 4.0))

        title_block_text = [
            _text(project_name or title_block_info.get("project_name", ""), margin + 4.0, margin + title_height - 10.0, 5.0),
            _text(detail_name or title_block_info.get("detail_name", ""), margin + 4.0, margin + title_height - 19.0, 4.0),
            _text("Scale 1:{0:g}".format(scale), sheet_width - margin - 116.0, margin + title_height - 10.0, 4.0),
            _text(str(title_block_info.get("sheet_number", "")), sheet_width - margin - 116.0, margin + title_height - 19.0, 4.0),
        ]

        report_lines = _report_lines(metadata, combined_report)
        cursor_y = margin + title_height - 29.0
        for line in report_lines:
            report_text_block.append(_text(line, margin + 4.0, cursor_y, 3.5))
            cursor_y -= 5.0

    named_view_data = _named_view_data(overall_bbox, _coerce_bool(create_named_views, True))
    print_ready_groups = {
        "top_view_group": {
            "viewport": viewport_defs["NODE_TOP"],
            "geometry": geometry_objects,
            "annotations": annotation_objects,
        },
        "front_view_group": {
            "viewport": viewport_defs["NODE_FRONT"],
            "geometry": geometry_objects,
            "annotations": annotation_objects,
        },
        "section_view_group": {
            "viewport": viewport_defs["NODE_SECTION"],
            "geometry": geometry_objects,
            "annotations": annotation_objects,
        },
        "isometric_view_group": {
            "viewport": viewport_defs["NODE_ISO"],
            "geometry": geometry_objects,
            "annotations": annotation_objects,
        },
        "report_group": report_text_block,
        "title_block_group": {
            "curves": title_block_curves,
            "text": title_block_text,
        },
    }

    return {
        "sheet_boundary": sheet_boundary,
        "viewport_guides": viewport_guides,
        "title_block_curves": title_block_curves,
        "title_block_text": title_block_text,
        "view_labels": view_labels,
        "report_text_block": report_text_block,
        "print_ready_groups": print_ready_groups,
        "named_view_data": named_view_data,
        "metadata": metadata,
        "messages": [
            "named_view_data is emitted as a side-effect-free descriptor set; bake or consume it downstream to create Rhino named views."
        ],
    }
