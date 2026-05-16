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
SOURCE_UNITS = "millimeters"

SOURCE_BASE_CENTER = (95980.17681639228, 13550.621533456946, 0.0)
SOURCE_FULL_PLATE_CENTER = (95060.8525, 11651.119, 0.0)

REFERENCE = {
    "base_length": 800.0,
    "base_width": 800.0,
    "base_thickness": 20.0,
    "hole_diameter": 50.0,
    "hole_spacing": (295.393, 292.9625),
    "edge_spacing": (104.607, 104.607, 107.037, 107.038),
    "omit_center_hole": False,
    "plates": [
        {
            "source_index": 20,
            "center": (95993.783, 12882.555, 301.227),
            "length": 287.302,
            "width": 140.0,
            "thickness": 10.0,
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
    return 2 if value == 2 else 1


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

        specs.append(
            {
                "source_index": index,
                "center": center,
                "length": length,
                "width": width,
                "thickness": thickness,
                "azimuth_deg": azimuth,
                "altitude_deg": inclination,
                "inclination_deg": inclination,
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
        local = _reference_local_point(spec["center"])
        next_spec = dict(spec)
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
            spec["center"][0] - SOURCE_FULL_PLATE_CENTER[0],
            spec["center"][1] - SOURCE_FULL_PLATE_CENTER[1],
            spec["center"][2] - SOURCE_FULL_PLATE_CENTER[2],
        )
        next_spec = dict(spec)
        next_spec["center"] = _point_on_plane(plane, local)
        specs.append(next_spec)
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


def _point_from_plane_local(plane, local_xyz):
    if rg is None or plane is None:
        return local_xyz
    point = rg.Point3d(plane.Origin)
    point += plane.XAxis * float(local_xyz[0])
    point += plane.YAxis * float(local_xyz[1])
    point += plane.ZAxis * float(local_xyz[2])
    return point


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
        center = _point_from_value(_list_at(input_centers, index), base.get("center"))
        base["center"] = center
        base["azimuth_deg"] = _float_at(plate_azimuths, index, base.get("azimuth_deg"))

        source_altitude = base.get("altitude_deg", base.get("inclination_deg"))
        timber_altitude = _timber_altitude_at(timber_elements, plate_timber_indices, index)
        altitude = timber_altitude if timber_altitude is not None else source_altitude
        explicit_altitude = _float_at(plate_altitudes, index, None)
        legacy_inclination = _float_at(plate_inclinations, index, None)
        if explicit_altitude is not None:
            altitude = explicit_altitude
        elif legacy_inclination is not None:
            altitude = legacy_inclination
        base["altitude_deg"] = altitude
        base["inclination_deg"] = altitude

        base["length"] = _float_at(plate_lengths, index, base.get("length"))
        base["width"] = _float_at(plate_widths, index, base.get("width"))
        base["thickness"] = _float_at(plate_thicknesses, index, base.get("thickness"))
        base["timber_index"] = _int_at(plate_timber_indices, index, index if timber_altitude is not None else None)
        base["altitude_source"] = (
            "override"
            if explicit_altitude is not None or legacy_inclination is not None
            else "timber_element"
            if timber_altitude is not None
            else "rhino_geometry"
        )
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
    result = []
    for index in range(count):
        base = dict(specs[index] if index < len(specs) else specs[-1])
        target_plate = plate_specs[index] if index < len(plate_specs) else plate_specs[-1]
        base["center"] = _point_from_value(_list_at(input_centers, index), base.get("center"))
        base["azimuth_deg"] = target_plate.get("azimuth_deg", base.get("azimuth_deg"))
        base["altitude_deg"] = target_plate.get("altitude_deg", target_plate.get("inclination_deg", base.get("altitude_deg")))
        base["inclination_deg"] = target_plate.get("inclination_deg", base.get("inclination_deg"))
        base["straight_length"] = target_plate.get("length", base.get("straight_length", base.get("length")))
        base["width"] = target_plate.get("width", base.get("width"))
        base["thickness"] = target_plate.get("thickness", base.get("thickness"))
        default_full_length = base.get("full_length", base.get("length"))
        base["full_length"] = _float_at(plate_full_lengths, index, default_full_length)
        base["length"] = base["full_length"]
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
    count = min(len(source_hole_specs), len(source_full_plate_specs), len(target_full_plate_specs))
    for index in range(count):
        source_hole = dict(source_hole_specs[index])
        source_spec = source_full_plate_specs[index]
        target_spec = target_full_plate_specs[index]
        source_hole["center"] = _transform_plate_point(
            source_hole["center"],
            source_spec,
            target_spec,
            index,
        )
        result.append(source_hole)
    return result


def _full_plate_transform_changed(source_full_plate_specs, target_full_plate_specs, tolerance=1e-6):
    count = min(len(source_full_plate_specs or []), len(target_full_plate_specs or []))
    for index in range(count):
        scale_x, scale_y, scale_z = _full_plate_transform_scales(
            source_full_plate_specs[index],
            target_full_plate_specs[index],
            index,
        )
        if (
            abs(scale_x - 1.0) > tolerance
            or abs(scale_y - 1.0) > tolerance
            or abs(scale_z - 1.0) > tolerance
        ):
            return True
    return False


def _plate_hole_specs_from_inputs(
    plane,
    source_hole_specs,
    source_full_plate_specs,
    target_full_plate_specs,
    plate_hole_centers=None,
    plate_hole_rows=None,
    plate_holes_per_row=None,
    plate_hole_diameters=None,
    plate_hole_row_spacings=None,
    plate_hole_pitches=None,
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

    input_centers = _flatten_values(plate_hole_centers)
    count = max(
        len(specs),
        len(input_centers),
        len(_flatten_values(plate_hole_rows)),
        len(_flatten_values(plate_holes_per_row)),
        len(_flatten_values(plate_hole_diameters)),
        len(_flatten_values(plate_hole_row_spacings)),
        len(_flatten_values(plate_hole_pitches)),
    )
    if count <= 0:
        return []

    result = []
    for index in range(count):
        base = dict(specs[index] if index < len(specs) else specs[-1])
        base["center"] = _point_from_value(_list_at(input_centers, index), base.get("center"))
        base["row_count"] = _row_count_at(plate_hole_rows, index, base.get("row_count", 1))
        base["holes_per_row"] = max(1, _int_at(plate_holes_per_row, index, base.get("holes_per_row", 4)))
        base["diameter"] = _float_at(plate_hole_diameters, index, base.get("diameter", 37.125))
        base["row_spacing"] = _float_at(plate_hole_row_spacings, index, base.get("row_spacing", 0.0))
        base["pitch"] = _float_at(plate_hole_pitches, index, base.get("pitch", 74.25))
        result.append(base)
    return result


def _stiffener_specs_from_inputs(
    plane,
    ref_with_stiffeners_brep=None,
    ref_stiffeners=None,
    stiffener_centers=None,
    stiffener_azimuths=None,
    stiffener_lengths=None,
    stiffener_widths=None,
    stiffener_heights=None,
    stiffener_low_heights=None,
    stiffener_thicknesses=None,
):
    if rg is not None and ref_stiffeners:
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


def _build_base_outline(plane, base_length, base_width):
    if rg is None:
        return None
    rectangle = rg.Rectangle3d(
        plane,
        rg.Interval(-0.5 * base_length, 0.5 * base_length),
        rg.Interval(-0.5 * base_width, 0.5 * base_width),
    )
    return rectangle.ToNurbsCurve()


def _build_hole_centers(plane, base_length, base_width, edge_spacing, hole_spacing, omit_center):
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
    center = hole_spec.get("center")
    if rg is None or center is None:
        return []

    plate_plane = _plate_plane(
        plate_spec["center"],
        plate_spec["azimuth_deg"],
        plate_spec["inclination_deg"],
    )
    row_offsets = [0.0] if row_count == 1 else [-0.5 * row_spacing, 0.5 * row_spacing]
    long_offsets = [
        (index - 0.5 * (holes_per_row - 1)) * pitch
        for index in range(holes_per_row)
    ]
    centers = []
    for row_offset in row_offsets:
        for long_offset in long_offsets:
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
    depth = max((plate_spec.get("thickness") or 1.0) * 4.0, diameter * 2.0, 1.0)
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
            plug_margin=0.05,
        )
        _target_centers, _target_curves, cutters, _target_per_plate = _plate_hole_geometry(
            [target_spec],
            [plate_spec],
        )

        blank = brep
        if plugs:
            try:
                unioned = rg.Brep.CreateBooleanUnion([brep] + plugs, 0.01)
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
                difference = rg.Brep.CreateBooleanDifference(blank, cutters, 0.01)
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


def _build_base_plate(plane, outline, hole_curves, base_length, base_width, base_thickness):
    if rg is None:
        return None
    if base_thickness and base_thickness > 0.0:
        box = rg.Box(
            plane,
            rg.Interval(-0.5 * base_length, 0.5 * base_length),
            rg.Interval(-0.5 * base_width, 0.5 * base_width),
            rg.Interval(0.0, base_thickness),
        )
        return box.ToBrep()

    curves = [outline] + [curve for curve in hole_curves if curve is not None]
    try:
        breps = rg.Brep.CreatePlanarBreps(curves, 0.01)
        if breps:
            return breps[0]
    except Exception:
        pass
    return outline


def _build_plate(spec):
    if rg is None:
        return None, None
    plane = _plate_plane(spec["center"], spec["azimuth_deg"], spec["inclination_deg"])
    long_axis = plane.XAxis
    box = rg.Box(
        plane,
        rg.Interval(-0.5 * spec["length"], 0.5 * spec["length"]),
        rg.Interval(-0.5 * spec["width"], 0.5 * spec["width"]),
        rg.Interval(-0.5 * spec["thickness"], 0.5 * spec["thickness"]),
    )
    start = spec["center"] - long_axis * (0.5 * spec["length"])
    end = spec["center"] + long_axis * (0.5 * spec["length"])
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
        }
    )[0]


def _transform_full_plate_breps(ref_full_plate_breps, source_specs, target_specs):
    refs = _flatten_values(ref_full_plate_breps)
    transformed = []
    for index, target in enumerate(target_specs):
        if index >= len(refs):
            break
        source = source_specs[index] if index < len(source_specs) else source_specs[-1]
        source_plane = source.get("plane")
        target_plane = _plate_plane(target["center"], target["azimuth_deg"], target["inclination_deg"])
        scale_x, scale_y, scale_z = _full_plate_transform_scales(source, target, index)
        brep = _transform_between_planes(refs[index], source_plane, target_plane, scale_x, scale_y, scale_z)
        if brep is not None:
            transformed.append(brep)
    return transformed


def _build_stiffener(spec):
    if rg is None:
        return None
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
            joined = rg.Brep.JoinBreps(pieces, 0.01)
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
            "Plate {0}: L={1:.1f}, W={2:.1f}, T={3:.1f}, az={4:.2f} deg, alt={5:.2f} deg ({6})".format(
                index,
                spec.get("length") or 0.0,
                spec.get("width") or 0.0,
                spec.get("thickness") or 0.0,
                spec.get("azimuth_deg") or 0.0,
                spec.get("altitude_deg", spec.get("inclination_deg")) or 0.0,
                spec.get("altitude_source") or "rhino_geometry",
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


def run(
    base_plane=None,
    ref_plates=None,
    ref_full_plate_breps=None,
    ref_with_stiffeners_brep=None,
    ref_base_outline=None,
    ref_hole_curves=None,
    ref_stiffeners=None,
    timber_elements=None,
    plate_timber_indices=None,
    base_length=None,
    base_width=None,
    base_thickness=None,
    plate_centers=None,
    plate_azimuths=None,
    plate_altitudes=None,
    plate_inclinations=None,
    plate_lengths=None,
    plate_full_lengths=None,
    plate_widths=None,
    plate_thicknesses=None,
    plate_full_centers=None,
    plate_hole_centers=None,
    plate_hole_rows=None,
    plate_holes_per_row=None,
    plate_hole_diameters=None,
    plate_hole_row_spacings=None,
    plate_hole_pitches=None,
    stiffener_centers=None,
    stiffener_azimuths=None,
    stiffener_lengths=None,
    stiffener_widths=None,
    stiffener_heights=None,
    stiffener_low_heights=None,
    stiffener_thicknesses=None,
    edge_spacing=None,
    hole_spacing=None,
    hole_diameter=None,
    hole_centers=None,
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
            "verification": {},
            "params": {},
        }

    plane = _as_plane(base_plane, reference_outline=ref_base_outline)
    base_length_value = _coerce_float(base_length, None)
    base_width_value = _coerce_float(base_width, None)
    base_thickness = _coerce_float(base_thickness, REFERENCE["base_thickness"])
    hole_diameter_value = _coerce_float(hole_diameter, None)
    include_stiffeners_override = _coerce_optional_bool(include_stiffeners)
    if include_stiffeners_override is None:
        include_stiffeners = bool(ref_stiffeners or ref_with_stiffeners_brep)
    else:
        include_stiffeners = include_stiffeners_override
    omit_center = _coerce_bool(omit_center_hole, True)
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

    base_length = _coerce_float(base_length_value, REFERENCE["base_length"])
    base_width = _coerce_float(base_width_value, REFERENCE["base_width"])

    plate_specs = _plate_specs_from_inputs(
        plane,
        reference_plates=ref_plates,
        timber_elements=timber_elements,
        plate_timber_indices=plate_timber_indices,
        plate_centers=plate_centers,
        plate_azimuths=plate_azimuths,
        plate_altitudes=plate_altitudes,
        plate_inclinations=plate_inclinations,
        plate_lengths=plate_lengths,
        plate_widths=plate_widths,
        plate_thicknesses=plate_thicknesses,
    )
    full_plate_specs = _full_plate_specs_from_inputs(
        plane,
        ref_full_plate_breps,
        plate_specs,
        plate_full_centers=plate_full_centers,
        plate_full_lengths=plate_full_lengths,
    )

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
    plate_hole_specs = _plate_hole_specs_from_inputs(
        plane,
        source_plate_hole_specs,
        source_full_specs,
        full_plate_specs,
        plate_hole_centers=plate_hole_centers,
        plate_hole_rows=plate_hole_rows,
        plate_holes_per_row=plate_holes_per_row,
        plate_hole_diameters=plate_hole_diameters,
        plate_hole_row_spacings=plate_hole_row_spacings,
        plate_hole_pitches=plate_hole_pitches,
    )
    plate_hole_warnings = []
    for index, spec in enumerate(plate_hole_specs):
        if spec.get("row_count") == 2 and (spec.get("row_spacing") or 0.0) <= 0.0:
            plate_hole_warnings.append(
                "Plate {0}: two-row hole pattern has zero row spacing.".format(index)
            )
    plate_hole_override_connected = any(
        _has_values(value)
        for value in (
            plate_hole_centers,
            plate_hole_rows,
            plate_holes_per_row,
            plate_hole_diameters,
            plate_hole_row_spacings,
            plate_hole_pitches,
        )
    )
    plate_body_scaled = _full_plate_transform_changed(source_full_specs, full_plate_specs)

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
            plane,
            base_length,
            base_width,
            edge_spacing_tuple,
            hole_spacing_pair,
            omit_center,
        )
        hole_specs = _hole_specs_from_centers(centers, hole_diameter)

    base_outline = _build_base_outline(plane, base_length, base_width)
    hole_curves = [_build_hole_curve(center, plane, hole_diameter) for center in centers]
    base_plate = _build_base_plate(
        plane,
        base_outline,
        hole_curves,
        base_length,
        base_width,
        base_thickness,
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
        full_plate_breps = [
            brep for brep in (_build_full_plate(spec) for spec in full_plate_specs)
            if brep is not None
        ]

    plate_hole_centers_built, plate_hole_curves, plate_hole_cutters, _per_plate_hole_centers = _plate_hole_geometry(
        plate_hole_specs,
        full_plate_specs,
    )
    repattern_messages = []
    repattern_needed = plate_hole_override_connected or plate_body_scaled or not ref_full_plate_breps
    if full_plate_breps and repattern_needed:
        full_plate_breps, repattern_messages = _repattern_full_plate_breps(
            full_plate_breps,
            transformed_source_plate_hole_specs,
            plate_hole_specs,
            full_plate_specs,
        )

    stiffener_breps = []
    stiffener_specs = []
    if include_stiffeners:
        stiffener_specs = _stiffener_specs_from_inputs(
            plane,
            ref_with_stiffeners_brep=ref_with_stiffeners_brep,
            ref_stiffeners=ref_stiffeners,
            stiffener_centers=stiffener_centers,
            stiffener_azimuths=stiffener_azimuths,
            stiffener_lengths=stiffener_lengths,
            stiffener_widths=stiffener_widths,
            stiffener_heights=stiffener_heights,
            stiffener_low_heights=stiffener_low_heights,
            stiffener_thicknesses=stiffener_thicknesses,
        )
        if rg is not None and ref_stiffeners:
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

    verification = _verify_breps(base_plate, full_plate_breps, stiffener_breps)
    verification["plate_hole_pattern_overridden"] = plate_hole_override_connected
    verification["plate_body_scaled"] = plate_body_scaled
    verification["plate_hole_repatterned"] = bool(repattern_needed and full_plate_breps)
    verification["plate_hole_cutter_count"] = len(plate_hole_cutters)
    verification["messages"].extend(plate_hole_warnings)
    verification["messages"].extend(repattern_messages)

    params = {
        "source_note": SOURCE_NOTE,
        "units": SOURCE_UNITS,
        "base_length": base_length,
        "base_width": base_width,
        "base_thickness": base_thickness,
        "edge_spacing": edge_spacing_tuple,
        "hole_spacing": hole_spacing_pair,
        "hole_diameter": hole_diameter,
        "hole_centers": centers,
        "plate_specs": plate_specs,
        "full_plate_specs": full_plate_specs,
        "plate_hole_specs": plate_hole_specs,
        "plate_hole_centers": plate_hole_centers_built,
        "timber_element_count": len(_flatten_values(timber_elements)),
        "include_stiffeners": include_stiffeners,
        "stiffener_specs": stiffener_specs,
        "stiffener_count": len(stiffener_breps),
        "verification": verification,
    }

    return {
        "summary_text": _summarize(
            plate_specs,
            full_plate_specs,
            hole_specs,
            plate_hole_specs,
            include_stiffeners,
            base_length,
            base_width,
            verification,
        ),
        "base_plate": base_plate,
        "base_outline": base_outline,
        "plate_breps": plate_breps,
        "full_plate_breps": full_plate_breps,
        "plate_axes": plate_axes,
        "hole_curves": hole_curves,
        "hole_centers": centers,
        "hole_specs": hole_specs,
        "plate_hole_curves": plate_hole_curves,
        "plate_hole_centers": plate_hole_centers_built,
        "plate_hole_specs": plate_hole_specs,
        "plate_hole_cutters": plate_hole_cutters,
        "stiffener_breps": stiffener_breps,
        "verification": verification,
        "params": params,
    }
