"""
Base plate calculations module.

Includes two layers:
1) geometric validation + iterative bottom-face adjustment along member axis
2) full Part 1/2/3 engineering formulas from the provided GH script

The return payload exposes GH-compatible output keys:
- a..n
- pass_fail_summary
- utilization_values
- combined_report / weld_report / anchor_report
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import Rhino.Geometry as rg  # type: ignore
except Exception:
    rg = None


Point3 = Tuple[float, float, float]


@dataclass
class ValidationConfig:
    min_allowable_clearance: float = 80.0
    shift_step: float = 10.0
    max_shift: float = 400.0
    base_plate_min_thickness: float = 10.0
    base_plate_max_thickness: float = 60.0


@dataclass
class MemberValidationResult:
    member_id: str
    member_index: int
    group: Optional[int]
    passed: bool
    initial_min_corner_clearance: float
    final_min_corner_clearance: float
    adjustment_along_member: float
    checks: Dict[str, bool]
    messages: List[str]


def _dot(a: Point3, b: Point3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _add(a: Point3, b: Point3) -> Point3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Point3, b: Point3) -> Point3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(v: Point3, s: float) -> Point3:
    return (v[0] * s, v[1] * s, v[2] * s)


def _length(v: Point3) -> float:
    return math.sqrt(_dot(v, v))


def _distance(a: Point3, b: Point3) -> float:
    return _length(_sub(a, b))


def _min_corner_distance(corners_a: Sequence[Point3], corners_b: Sequence[Point3]) -> float:
    min_dist = float("inf")
    for pa in corners_a:
        for pb in corners_b:
            d = _distance(pa, pb)
            if d < min_dist:
                min_dist = d
    return min_dist


def _shift_corners(corners: Sequence[Point3], axis: Point3, distance: float) -> List[Point3]:
    delta = _scale(axis, distance)
    return [_add(c, delta) for c in corners]


def _member_map(payload: Dict[str, object]) -> Dict[str, Dict[str, object]]:
    result: Dict[str, Dict[str, object]] = {}
    for m in payload.get("members", []):
        if isinstance(m, dict) and "member_id" in m:
            result[str(m["member_id"])] = m
    return result


def _plate_records(payload: Dict[str, object]) -> List[Dict[str, object]]:
    records = payload.get("base_plates", [])
    return [r for r in records if isinstance(r, dict)]


def _is_plate_thickness_ok(thickness: float, cfg: ValidationConfig) -> bool:
    return cfg.base_plate_min_thickness <= thickness <= cfg.base_plate_max_thickness


def _clearance_against_previous(corners: Sequence[Point3], previous_corners: Iterable[Sequence[Point3]]) -> float:
    min_clearance = float("inf")
    for other in previous_corners:
        d = _min_corner_distance(corners, other)
        if d < min_clearance:
            min_clearance = d
    return min_clearance if min_clearance != float("inf") else 1e9


def _center_of_points(points: Sequence[Point3]) -> Point3:
    if not points:
        return (0.0, 0.0, 0.0)
    n = float(len(points))
    return (
        sum(p[0] for p in points) / n,
        sum(p[1] for p in points) / n,
        sum(p[2] for p in points) / n,
    )


def _rectangle_corners(center: Point3, x_axis: Point3, y_axis: Point3, lx: float, ly: float) -> List[Point3]:
    hx = 0.5 * float(lx)
    hy = 0.5 * float(ly)
    return [
        _add(center, _add(_scale(x_axis, hx), _scale(y_axis, hy))),
        _add(center, _add(_scale(x_axis, -hx), _scale(y_axis, hy))),
        _add(center, _add(_scale(x_axis, -hx), _scale(y_axis, -hy))),
        _add(center, _add(_scale(x_axis, hx), _scale(y_axis, -hy))),
    ]


def _run_geometric_validation_pass(
    members: Dict[str, Dict[str, object]],
    plates: Sequence[Dict[str, object]],
    cfg: ValidationConfig,
) -> Tuple[List[MemberValidationResult], List[Dict[str, object]]]:
    results: List[MemberValidationResult] = []
    adjusted_plates: List[Dict[str, object]] = []
    previous_adjusted: List[Sequence[Point3]] = []

    for plate in sorted(plates, key=lambda x: int(x.get("member_index") or 0)):
        member_id = str(plate.get("member_id"))
        member = members.get(member_id, {})

        validation, adjusted_corners, adjustment = _validate_and_adjust_single(
            plate,
            member,
            previous_adjusted,
            cfg,
        )
        results.append(validation)
        previous_adjusted.append(adjusted_corners)

        plate_copy = dict(plate)
        plate_copy["corners_adjusted"] = adjusted_corners
        plate_copy["adjustment_along_member"] = adjustment
        adjusted_plates.append(plate_copy)

    return results, adjusted_plates


def _apply_engineering_sizing_to_plates(
    plates: Sequence[Dict[str, object]],
    sizing: Dict[str, object],
) -> List[Dict[str, object]]:
    target_length = float(sizing.get("plate_length") or 0.0)
    target_width = float(sizing.get("plate_width") or 0.0)
    target_thickness = float(sizing.get("plate_thickness") or 0.0)

    out: List[Dict[str, object]] = []
    for plate in plates:
        plate_copy = dict(plate)

        if target_length > 0.0:
            plate_copy["length"] = target_length
        if target_width > 0.0:
            plate_copy["width"] = target_width
        if target_thickness > 0.0:
            plate_copy["thickness"] = target_thickness

        corners_source = plate_copy.get("corners_adjusted") or plate_copy.get("corners") or []
        corners_points: List[Point3] = []
        for c in corners_source:
            if isinstance(c, Sequence) and len(c) >= 3:
                corners_points.append((float(c[0]), float(c[1]), float(c[2])))

        center_raw = plate_copy.get("center")
        if isinstance(center_raw, Sequence) and len(center_raw) >= 3:
            center = (float(center_raw[0]), float(center_raw[1]), float(center_raw[2]))
        else:
            center = _center_of_points(corners_points)

        x_raw = plate_copy.get("x_axis") or (1.0, 0.0, 0.0)
        y_raw = plate_copy.get("y_axis") or (0.0, 1.0, 0.0)
        if not (isinstance(x_raw, Sequence) and len(x_raw) >= 3):
            x_raw = (1.0, 0.0, 0.0)
        if not (isinstance(y_raw, Sequence) and len(y_raw) >= 3):
            y_raw = (0.0, 1.0, 0.0)
        x_axis = (float(x_raw[0]), float(x_raw[1]), float(x_raw[2]))
        y_axis = (float(y_raw[0]), float(y_raw[1]), float(y_raw[2]))

        length = float(plate_copy.get("length") or 0.0)
        width = float(plate_copy.get("width") or 0.0)
        if length > 0.0 and width > 0.0:
            rec_corners = _rectangle_corners(center, x_axis, y_axis, length, width)
            plate_copy["center"] = center
            plate_copy["corners"] = rec_corners
            plate_copy.pop("corners_adjusted", None)

        out.append(plate_copy)

    return out


def _build_synced_geometry_payload(
    geometry_payload: Dict[str, object],
    adjusted_plates: Sequence[Dict[str, object]],
    sync_iterations: int,
    sync_applied: bool,
) -> Dict[str, object]:
    synced = deepcopy(geometry_payload)
    synced_base_plates: List[Dict[str, object]] = []
    for plate in adjusted_plates:
        plate_copy = dict(plate)
        if "corners_adjusted" in plate_copy:
            plate_copy["corners"] = plate_copy["corners_adjusted"]
            plate_copy.pop("corners_adjusted", None)
        synced_base_plates.append(plate_copy)

    synced["base_plates"] = synced_base_plates
    meta = dict(synced.get("metadata") or {})
    meta["sizing_sync_applied"] = bool(sync_applied)
    meta["sizing_sync_iterations"] = int(sync_iterations)
    synced["metadata"] = meta
    return synced


def _validate_and_adjust_single(
    plate: Dict[str, object],
    member: Dict[str, object],
    previous_adjusted: List[Sequence[Point3]],
    cfg: ValidationConfig,
) -> Tuple[MemberValidationResult, List[Point3], float]:
    member_id = str(plate.get("member_id"))
    index = int(plate.get("member_index") or 0)
    group = plate.get("group")

    direction_raw = member.get("direction") or (0.0, 0.0, 1.0)
    if not (isinstance(direction_raw, Sequence) and len(direction_raw) >= 3):
        direction = (0.0, 0.0, 1.0)
    else:
        direction = (float(direction_raw[0]), float(direction_raw[1]), float(direction_raw[2]))

    corners_raw = plate.get("corners") or []
    corners: List[Point3] = []
    for c in corners_raw:
        if isinstance(c, Sequence) and len(c) >= 3:
            corners.append((float(c[0]), float(c[1]), float(c[2])))

    thickness = float(plate.get("thickness") or 0.0)

    initial_clearance = _clearance_against_previous(corners, previous_adjusted)
    adjusted = list(corners)
    adjustment = 0.0

    if initial_clearance < cfg.min_allowable_clearance:
        attempts = int(math.ceil(cfg.max_shift / max(cfg.shift_step, 1e-6)))
        for _ in range(attempts):
            adjustment += cfg.shift_step
            adjusted = _shift_corners(corners, direction, adjustment)
            current_clearance = _clearance_against_previous(adjusted, previous_adjusted)
            if current_clearance >= cfg.min_allowable_clearance:
                break

    final_clearance = _clearance_against_previous(adjusted, previous_adjusted)

    checks = {
        "plate_thickness_ok": _is_plate_thickness_ok(thickness, cfg),
        "corner_clearance_ok": final_clearance >= cfg.min_allowable_clearance,
        "adjustment_within_limit": adjustment <= cfg.max_shift + 1e-6,
    }

    passed = all(checks.values())
    messages: List[str] = []
    if not checks["plate_thickness_ok"]:
        messages.append(
            "Plate thickness {0:.1f} mm outside [{1:.1f}, {2:.1f}] mm".format(
                thickness, cfg.base_plate_min_thickness, cfg.base_plate_max_thickness
            )
        )
    if not checks["corner_clearance_ok"]:
        messages.append(
            "Minimum tip-corner clearance {0:.1f} mm is below required {1:.1f} mm".format(
                final_clearance, cfg.min_allowable_clearance
            )
        )
    if adjustment > 0.0:
        messages.append("Bottom face shifted by {0:.1f} mm along member axis".format(adjustment))

    return (
        MemberValidationResult(
            member_id=member_id,
            member_index=index,
            group=int(group) if group is not None else None,
            passed=passed,
            initial_min_corner_clearance=initial_clearance,
            final_min_corner_clearance=final_clearance,
            adjustment_along_member=adjustment,
            checks=checks,
            messages=messages,
        ),
        adjusted,
        adjustment,
    )


def _round_up_stock(x: float, stock: Optional[Sequence[float]] = None) -> float:
    candidates = stock or [8, 10, 12, 15, 20, 25, 30, 35, 40]
    for s in candidates:
        if x <= s:
            return float(s)
    return float(candidates[-1])


def _truthy_override(value: object) -> bool:
    try:
        return value is not None and float(value) > 0
    except Exception:
        return False


def _bolt_area(d: float) -> float:
    return math.pi * d * d / 4.0


def _utilization(demand: float, capacity: float) -> float:
    if capacity is None or capacity <= 0:
        return 999.0
    return demand / capacity


def _passfail(util: float) -> str:
    return "OK" if util <= 1.0 else "NG"


def _make_box(center: Point3, x: float, y: float, z: float):
    if rg is None:
        return None
    plane = rg.Plane(rg.Point3d(center[0], center[1], center[2]), rg.Vector3d.XAxis, rg.Vector3d.YAxis)
    return rg.Box(
        plane,
        rg.Interval(-x / 2.0, x / 2.0),
        rg.Interval(-y / 2.0, y / 2.0),
        rg.Interval(-z / 2.0, z / 2.0),
    ).ToBrep()


def _make_cylinder_between(p0: Point3, p1: Point3, radius: float):
    if rg is None:
        return None
    p0p = rg.Point3d(p0[0], p0[1], p0[2])
    p1p = rg.Point3d(p1[0], p1[1], p1[2])
    axis = p1p - p0p
    length = axis.Length
    if length <= 0:
        return None
    mid = p0p + axis * 0.5
    plane = rg.Plane(mid, axis)
    cyl = rg.Cylinder(rg.Circle(plane, radius), length)
    return cyl.ToBrep(True, True)


def _run_part2_weld_report(
    F_Ed_part1: float,
    Fu_steel: float,
    steel_grade: str,
    gamma_M2: float,
    weld_leg_size: float,
    beta_w: float,
    weld_effective_lengths: Sequence[float],
    weld_force_share: Optional[Sequence[float]],
    weld_eccentric_moment_Ed: float,
) -> Tuple[str, List[float], bool]:
    reports: List[str] = []
    utils: List[float] = []

    throat_a = 0.7 * weld_leg_size
    fvw_d = Fu_steel / (math.sqrt(3.0) * beta_w * gamma_M2)
    resistance_per_mm = throat_a * fvw_d / 1000.0

    if weld_force_share and hasattr(weld_force_share, "__iter__"):
        forces = [float(x) for x in weld_force_share]
    else:
        total_len = sum(float(L) for L in weld_effective_lengths) if weld_effective_lengths else 0.0
        forces = []
        for L in weld_effective_lengths:
            share = float(L) / total_len if total_len > 0 else 0.0
            forces.append(F_Ed_part1 * share)

    reports.append("PART 2: WELD CHECK REPORT")
    reports.append("Code basis: SIA 263 / EN 1993-1-8, fillet weld resistance check.")
    reports.append("Weld check active: TRUE")
    reports.append("Steel grade: {0}, Fu = {1:.0f} N/mm2".format(steel_grade, Fu_steel))
    reports.append("Fillet weld leg size s = {0:.1f} mm; effective throat a ~= 0.7s = {1:.1f} mm".format(weld_leg_size, throat_a))
    reports.append("Design weld shear strength fvw,d ~= {0:.1f} N/mm2".format(fvw_d))
    reports.append("Resistance per mm effective weld length ~= {0:.3f} kN/mm".format(resistance_per_mm))

    for idx, L in enumerate(weld_effective_lengths):
        Leff = float(L)
        Fed = forces[idx] if idx < len(forces) else 0.0
        Rrd = resistance_per_mm * Leff
        eta = _utilization(Fed, Rrd)
        utils.append(eta)
        reports.append(
            "Weld W{0}: L_eff = {1:.1f} mm, F_Ed = {2:.2f} kN, F_Rd = {3:.2f} kN, utilization = {4:.2f} [{5}]".format(
                idx + 1, Leff, Fed, Rrd, eta, _passfail(eta)
            )
        )

    if abs(weld_eccentric_moment_Ed) > 0:
        reports.append(
            "NOTE: Eccentric weld moment input M_Ed = {0:.2f} kNm. This template flags it but does not yet perform full weld-group polar inertia distribution.".format(
                weld_eccentric_moment_Ed
            )
        )
        reports.append("For final design, distribute direct shear plus torsion/moment to each weld segment using weld group elastic method.")
    else:
        reports.append("Eccentric weld moment input = 0. Direct force distribution only.")

    ok = all(u <= 1.0 for u in utils) if utils else True
    reports.append("Part 2 weld result: {0}".format("OK PRELIMINARY" if ok else "NG - INCREASE WELD SIZE/LENGTH OR REVISE LOAD PATH"))
    return "\n".join(reports), utils, ok


def _run_part3_anchor_report(
    V_lat_Ed: float,
    N_comp_Ed: float,
    N_tens_Ed: float,
    Mx_Ed: float,
    My_Ed: float,
    gamma_M2: float,
    base_plate_length: float,
    base_plate_width: float,
    base_plate_thickness: float,
    anchor_bolt_dia: float,
    anchor_bolt_count: int,
    anchor_bolt_Fub: float,
    anchor_pattern_x: float,
    anchor_pattern_y: float,
    screw_pier_rated_compression: float,
    screw_pier_rated_tension: float,
    screw_pier_rated_lateral: float,
    screw_pier_rated_moment: float,
) -> Tuple[str, List[float], bool]:
    reports: List[str] = []
    utils: List[float] = []

    reports.append("PART 3: BASE PLATE TO SCREW-PIER CAP / ANCHOR BOLT CHECK REPORT")
    reports.append("Code basis: SIA 263 / EN 1993 for steel plate/bolt checks; SIA 267 / EN 1997-1 + manufacturer data for screw-pier geotechnical resistance.")
    reports.append("No concrete anchorage checks are used in this module.")
    reports.append("Anchor/screw-pier check active: TRUE")

    V_total = math.sqrt((V_lat_Ed) ** 2 + 0.0)
    N_comp = abs(N_comp_Ed)
    N_tens = abs(N_tens_Ed)
    M_total = math.sqrt(Mx_Ed ** 2 + My_Ed ** 2)

    A_anchor = _bolt_area(anchor_bolt_dia)
    Fv_anchor_single = 0.6 * anchor_bolt_Fub * A_anchor / gamma_M2 / 1000.0
    Fv_anchor_group = Fv_anchor_single * anchor_bolt_count
    util_anchor_shear = _utilization(V_total, Fv_anchor_group)
    utils.append(util_anchor_shear)

    As_tension = 0.75 * A_anchor
    Ft_anchor_per = 0.9 * anchor_bolt_Fub * As_tension / gamma_M2 / 1000.0
    Ft_anchor_group = Ft_anchor_per * anchor_bolt_count
    util_anchor_tension = _utilization(N_tens, Ft_anchor_group)
    utils.append(util_anchor_tension)

    interaction = util_anchor_shear ** 2 + util_anchor_tension ** 2
    utils.append(interaction)

    util_pier_comp = _utilization(N_comp, screw_pier_rated_compression)
    util_pier_tens = _utilization(N_tens, screw_pier_rated_tension)
    util_pier_lat = _utilization(V_total, screw_pier_rated_lateral)
    util_pier_mom = _utilization(M_total, screw_pier_rated_moment)
    utils.extend([util_pier_comp, util_pier_tens, util_pier_lat, util_pier_mom])

    reports.append("Base plate: {0:.0f} x {1:.0f} x {2:.0f} mm".format(base_plate_length, base_plate_width, base_plate_thickness))
    reports.append("Cap/anchor bolts: {0} x M{1:.0f}, Fub = {2:.0f} N/mm2".format(anchor_bolt_count, anchor_bolt_dia, anchor_bolt_Fub))
    reports.append("Bolt pattern: {0:.0f} x {1:.0f} mm".format(anchor_pattern_x, anchor_pattern_y))
    reports.append("Actions checked: Ncomp = {0:.2f} kN, Ntens = {1:.2f} kN, V = {2:.2f} kN, M = {3:.2f} kNm".format(N_comp, N_tens, V_total, M_total))

    reports.append("Steel cap bolt shear resistance group = {0:.2f} kN; utilization = {1:.2f} [{2}]".format(Fv_anchor_group, util_anchor_shear, _passfail(util_anchor_shear)))
    reports.append("Steel cap bolt tension resistance group ~= {0:.2f} kN; utilization = {1:.2f} [{2}]".format(Ft_anchor_group, util_anchor_tension, _passfail(util_anchor_tension)))
    reports.append("Steel cap bolt combined interaction etaV^2 + etaT^2 = {0:.2f} [{1}]".format(interaction, _passfail(interaction)))

    reports.append("Screw-pier rated compression = {0:.2f} kN; utilization = {1:.2f} [{2}]".format(screw_pier_rated_compression, util_pier_comp, _passfail(util_pier_comp)))
    reports.append("Screw-pier rated uplift/tension = {0:.2f} kN; utilization = {1:.2f} [{2}]".format(screw_pier_rated_tension, util_pier_tens, _passfail(util_pier_tens)))
    reports.append("Screw-pier rated lateral = {0:.2f} kN; utilization = {1:.2f} [{2}]".format(screw_pier_rated_lateral, util_pier_lat, _passfail(util_pier_lat)))
    reports.append("Screw-pier rated moment = {0:.2f} kNm; utilization = {1:.2f} [{2}]".format(screw_pier_rated_moment, util_pier_mom, _passfail(util_pier_mom)))

    if screw_pier_rated_compression >= 9999 or screw_pier_rated_tension >= 9999 or screw_pier_rated_lateral >= 9999:
        reports.append("WARNING: Screw-pier capacities appear to be default placeholders. Enter manufacturer/geotechnical rated compression, uplift, lateral, and moment capacities.")

    ok = all(u <= 1.0 for u in utils)
    reports.append("Part 3 result: {0}".format("OK PRELIMINARY" if ok else "NG - REVISE CAP BOLTS, BASE PLATE, OR SCREW-PIER SYSTEM"))
    return "\n".join(reports), utils, ok


def run_engineering_checks(overrides: Optional[Dict[str, object]] = None) -> Dict[str, object]:
    o = overrides or {}

    def g(name: str, default):
        return o[name] if name in o else default

    run_part2_weld_checks = bool(g("run_part2_weld_checks", True))
    run_part3_anchor_bolt_checks = bool(g("run_part3_anchor_bolt_checks", True))
    render_geometry = bool(g("render_geometry", True))

    b_timber = float(g("b_timber", 100.0))
    h_timber = float(g("h_timber", 140.0))
    rho_k = float(g("rho_k", 350.0))
    kmod = float(g("kmod", 0.8))
    gamma_M_timber = float(g("gamma_M_timber", 1.3))

    steel_grade = str(g("steel_grade", "S235"))
    Fy_steel = float(g("Fy_steel", 235.0))
    Fu_steel = float(g("Fu_steel", 360.0))
    gamma_M0 = float(g("gamma_M0", 1.0))
    gamma_M2 = float(g("gamma_M2", 1.25))

    bolt_dia = float(g("bolt_dia", 12.0))
    bolt_grade = str(g("bolt_grade", "4.6"))
    bolt_Fub = float(g("bolt_Fub", 400.0))
    hole_clearance = float(g("hole_clearance", 1.0))
    bolt_hole_dia = float(g("bolt_hole_dia", bolt_dia + hole_clearance))

    rows = int(g("rows", 2))
    holes_per_row = int(g("holes_per_row", 2))

    N_comp_Ed = float(g("N_comp_Ed", 27.6))
    N_tens_Ed = float(g("N_tens_Ed", 7.9))
    V_lat_Ed = float(g("V_lat_Ed", 0.0))
    angle_from_vertical_deg = float(g("angle_from_vertical_deg", 32.5))
    angle_to_grain_deg = float(g("angle_to_grain_deg", 0.0))

    plate_thickness_in = g("plate_thickness", 0.0)
    pitch_parallel_in = g("pitch_parallel", 0.0)
    gage_perp_in = g("gage_perp", 0.0)
    end_distance_in = g("end_distance", 0.0)
    edge_distance_in = g("edge_distance", 0.0)
    plate_depth_in = g("plate_depth", 0.0)
    plate_length_in = g("plate_length", 0.0)

    washer_face_dia = float(g("washer_face_dia", 45.0))
    washer_recess_depth = float(g("washer_recess_depth", 25.0))
    plug_depth = float(g("plug_depth", 10.0))
    slot_clearance_each_side = float(g("slot_clearance_each_side", 1.5))
    slot_extra_length = float(g("slot_extra_length", 20.0))
    slot_extra_depth = float(g("slot_extra_depth", 2.0))

    clearance_between_converging_timbers = float(g("clearance_between_converging_timbers", 80.0))
    min_tool_clearance_preferred = float(g("min_tool_clearance_preferred", 80.0))
    min_tool_clearance_absolute = float(g("min_tool_clearance_absolute", 60.0))
    min_recess_edge_clear = float(g("min_recess_edge_clear", 15.0))
    min_bolt_axis_to_obstruction = float(g("min_bolt_axis_to_obstruction", 45.0))
    adjacent_obstruction_clearance = float(g("adjacent_obstruction_clearance", 9999.0))
    bolt_axis_to_obstruction = float(g("bolt_axis_to_obstruction", 9999.0))

    weld_leg_size = float(g("weld_leg_size", 6.0))
    beta_w = float(g("beta_w", 0.8))
    weld_effective_lengths = g("weld_effective_lengths", [100.0, 100.0, 120.0, 120.0])
    weld_force_share = g("weld_force_share", None)
    weld_eccentric_moment_Ed = float(g("weld_eccentric_moment_Ed", 0.0))

    base_plate_width = float(g("base_plate_width", 300.0))
    base_plate_length = float(g("base_plate_length", 300.0))
    base_plate_thickness = float(g("base_plate_thickness", 20.0))
    anchor_bolt_dia = float(g("anchor_bolt_dia", 16.0))
    anchor_bolt_count = int(g("anchor_bolt_count", 4))
    anchor_bolt_Fub = float(g("anchor_bolt_Fub", 400.0))
    anchor_pattern_x = float(g("anchor_pattern_x", 200.0))
    anchor_pattern_y = float(g("anchor_pattern_y", 200.0))

    screw_pier_rated_compression = float(g("screw_pier_rated_compression", 9999.0))
    screw_pier_rated_tension = float(g("screw_pier_rated_tension", 9999.0))
    screw_pier_rated_lateral = float(g("screw_pier_rated_lateral", 9999.0))
    screw_pier_rated_moment = float(g("screw_pier_rated_moment", 9999.0))
    Mx_Ed = float(g("Mx_Ed", 0.0))
    My_Ed = float(g("My_Ed", 0.0))

    t_plate_auto = max(15.0, _round_up_stock(0.8 * bolt_dia))
    t_plate = float(plate_thickness_in) if _truthy_override(plate_thickness_in) else t_plate_auto

    p_auto = max(7.0 * bolt_dia, 100.0)
    g_auto = max(5.0 * bolt_dia, 60.0)
    e1_auto = max(7.0 * bolt_dia, 110.0)
    e2_auto = max(3.5 * bolt_dia, 40.0)

    pitch_parallel = float(pitch_parallel_in) if _truthy_override(pitch_parallel_in) else p_auto
    gage_perp = float(gage_perp_in) if _truthy_override(gage_perp_in) else g_auto
    end_distance = float(end_distance_in) if _truthy_override(end_distance_in) else e1_auto
    edge_distance = float(edge_distance_in) if _truthy_override(edge_distance_in) else e2_auto

    plate_length = float(plate_length_in) if _truthy_override(plate_length_in) else (2.0 * end_distance + (holes_per_row - 1) * pitch_parallel)
    plate_depth = float(plate_depth_in) if _truthy_override(plate_depth_in) else (2.0 * edge_distance + (rows - 1) * gage_perp)

    slot_width = t_plate + 2.0 * slot_clearance_each_side
    slot_length = plate_length + slot_extra_length
    slot_depth = min(h_timber, plate_depth + slot_extra_depth)

    t_side = (b_timber - t_plate) / 2.0
    n_bolts = rows * holes_per_row

    F_axial_Ed = max(abs(N_comp_Ed), abs(N_tens_Ed))
    F_combined_Ed = math.sqrt(F_axial_Ed ** 2 + V_lat_Ed ** 2)
    F_Ed_part1 = F_combined_Ed
    V_Ed_per_bolt = F_Ed_part1 / max(n_bolts, 1)

    fh0k = 0.082 * (1.0 - 0.01 * bolt_dia) * rho_k
    angle_rad = math.radians(angle_to_grain_deg)
    angle_factor = max(0.65, math.cos(angle_rad) ** 2 + 0.65 * math.sin(angle_rad) ** 2)
    fhk = fh0k * angle_factor

    Rk_embed_per_bolt = 2.0 * fhk * t_side * bolt_dia / 1000.0
    group_factor = 0.90 if holes_per_row > 1 else 1.00
    Rd_embed_per_bolt = Rk_embed_per_bolt * kmod / gamma_M_timber
    Rd_embed_group = Rd_embed_per_bolt * n_bolts * group_factor
    util_embed = _utilization(F_Ed_part1, Rd_embed_group)

    A_bolt = _bolt_area(bolt_dia)
    Fv_Rd_single_plane = 0.6 * bolt_Fub * A_bolt / gamma_M2 / 1000.0
    Fv_Rd_double_shear_per_bolt = 2.0 * Fv_Rd_single_plane
    Fv_Rd_group = n_bolts * Fv_Rd_double_shear_per_bolt
    util_bolt_shear = _utilization(F_Ed_part1, Fv_Rd_group)

    bearing_capacity_per_bolt = 2.5 * bolt_dia * t_plate * Fu_steel / gamma_M2 / 1000.0
    bearing_group = bearing_capacity_per_bolt * n_bolts
    util_plate_bearing = _utilization(F_Ed_part1, bearing_group)

    A_net_plate = max(0.0, (plate_depth - rows * bolt_hole_dia) * t_plate)
    N_Rd_plate_net = A_net_plate * Fy_steel / gamma_M0 / 1000.0
    util_plate_net = _utilization(F_Ed_part1, N_Rd_plate_net)

    washer_area = math.pi * (washer_face_dia ** 2 - bolt_hole_dia ** 2) / 4.0
    fc90d_assumed = float(g("fc90d_assumed", 2.5))
    washer_capacity_per_face = washer_area * fc90d_assumed / 1000.0
    washer_capacity_group = 2.0 * washer_capacity_per_face * n_bolts
    util_washer_bearing = _utilization(F_Ed_part1, washer_capacity_group)

    remaining_timber_after_recess = (b_timber / 2.0) - washer_recess_depth
    recess_geometry_ok = remaining_timber_after_recess >= 25.0 and washer_face_dia <= max(45.0, h_timber - 2.0 * edge_distance + 10.0)

    if clearance_between_converging_timbers >= min_tool_clearance_preferred:
        installation_clearance_status = "OK"
    elif clearance_between_converging_timbers >= min_tool_clearance_absolute:
        installation_clearance_status = "TIGHT"
    else:
        installation_clearance_status = "NG"

    recess_edge_clear_ok = adjacent_obstruction_clearance >= min_recess_edge_clear
    bolt_axis_clear_ok = bolt_axis_to_obstruction >= min_bolt_axis_to_obstruction
    installation_preferred_ok = installation_clearance_status == "OK" and recess_edge_clear_ok and bolt_axis_clear_ok
    installation_absolute_ok = installation_clearance_status in ["OK", "TIGHT"] and recess_edge_clear_ok and bolt_axis_clear_ok

    geometry_plate_fits_depth = plate_depth <= h_timber
    geometry_side_thickness_ok = t_side >= 3.0 * bolt_dia
    geometry_slot_ok = slot_width <= b_timber and slot_depth <= h_timber
    geometry_ok = geometry_plate_fits_depth and geometry_side_thickness_ok and geometry_slot_ok and recess_geometry_ok and installation_absolute_ok

    part1_utils = {
        "timber_embedment": util_embed,
        "bolt_shear": util_bolt_shear,
        "plate_bearing": util_plate_bearing,
        "plate_net_section": util_plate_net,
        "washer_bearing": util_washer_bearing,
    }
    part1_strength_ok = all(u <= 1.0 for u in part1_utils.values())

    timber_brep = None
    plate_brep = None
    slot_cut_brep = None
    bolt_breps: List[object] = []
    hole_breps: List[object] = []
    washer_recess_breps: List[object] = []
    washer_face_circles: List[object] = []
    bolt_points: List[object] = []
    force_vectors: List[object] = []

    if render_geometry and rg is not None:
        timber_brep = _make_box((0.0, 0.0, 0.0), plate_length + 120.0, b_timber, h_timber)
        plate_brep = _make_box((0.0, 0.0, 0.0), plate_length, t_plate, plate_depth)
        slot_cut_brep = _make_box((0.0, 0.0, 0.0), slot_length, slot_width, slot_depth)

        x0 = -((holes_per_row - 1) * pitch_parallel) / 2.0
        z0 = -((rows - 1) * gage_perp) / 2.0

        for i in range(holes_per_row):
            for j in range(rows):
                x = x0 + i * pitch_parallel
                z = z0 + j * gage_perp
                pt = rg.Point3d(x, 0.0, z)
                bolt_points.append(pt)

                bolt_brep = _make_cylinder_between((x, -b_timber / 2.0 - 8.0, z), (x, b_timber / 2.0 + 8.0, z), bolt_dia / 2.0)
                hole_brep = _make_cylinder_between((x, -b_timber / 2.0 - 2.0, z), (x, b_timber / 2.0 + 2.0, z), bolt_hole_dia / 2.0)
                if bolt_brep is not None:
                    bolt_breps.append(bolt_brep)
                if hole_brep is not None:
                    hole_breps.append(hole_brep)

                y_outer1 = -b_timber / 2.0
                y_inner1 = y_outer1 + washer_recess_depth
                rec1 = _make_cylinder_between((x, y_outer1, z), (x, y_inner1, z), washer_face_dia / 2.0)

                y_outer2 = b_timber / 2.0
                y_inner2 = y_outer2 - washer_recess_depth
                rec2 = _make_cylinder_between((x, y_outer2, z), (x, y_inner2, z), washer_face_dia / 2.0)
                if rec1 is not None:
                    washer_recess_breps.append(rec1)
                if rec2 is not None:
                    washer_recess_breps.append(rec2)

                for side in [-1, 1]:
                    center = rg.Point3d(x, side * b_timber / 2.0, z)
                    plane = rg.Plane(center, rg.Vector3d.YAxis)
                    washer_face_circles.append(rg.Circle(plane, washer_face_dia / 2.0))

        ang = math.radians(angle_from_vertical_deg)
        v = rg.Vector3d(math.sin(ang), 0.0, math.cos(ang))
        v.Unitize()
        scale_vis = 3.0
        force_line = rg.Line(rg.Point3d(0.0, 0.0, 0.0), rg.Point3d(v.X * F_Ed_part1 * scale_vis, 0.0, v.Z * F_Ed_part1 * scale_vis))
        force_vectors.append(force_line)

    part1_report_lines: List[str] = []
    part1_report_lines.append("PART 1: TIMBER TO CONCEALED SLOTTED STEEL PLATE WITH THROUGH-BOLTS")
    part1_report_lines.append("Code basis: SIA 265 / EN 1995-1-1 dowel-type fastener concepts, plus SIA 263 / EN 1993 steel bolt/plate checks.")
    part1_report_lines.append("Fastener mode: through-bolts with recessed nuts/washers; not free smooth dowels.")
    part1_report_lines.append("Timber: {0:.0f} mm thick x {1:.0f} mm deep; rho_k = {2:.0f} kg/m3".format(b_timber, h_timber, rho_k))
    part1_report_lines.append("Steel plate: {0:.0f} mm {1}; Fy = {2:.0f} N/mm2".format(t_plate, steel_grade, Fy_steel))
    part1_report_lines.append("Bolts: {0} x M{1:.0f}, grade {2}, holes O{3:.0f} mm".format(n_bolts, bolt_dia, bolt_grade, bolt_hole_dia))
    part1_report_lines.append("Layout: {0} rows x {1} holes/row".format(rows, holes_per_row))
    part1_report_lines.append("Pitch along member p = {0:.0f} mm; gage across depth g = {1:.0f} mm".format(pitch_parallel, gage_perp))
    part1_report_lines.append("End distance e1 = {0:.0f} mm; edge distance e2 = {1:.0f} mm".format(end_distance, edge_distance))
    part1_report_lines.append("Plate length = {0:.0f} mm; plate depth = {1:.0f} mm".format(plate_length, plate_depth))
    part1_report_lines.append("Side timber thickness each side of plate = {0:.1f} mm".format(t_side))
    part1_report_lines.append("Design force used F_Ed = {0:.2f} kN".format(F_Ed_part1))
    part1_report_lines.append("Demand per bolt = {0:.2f} kN".format(V_Ed_per_bolt))

    part1_report_lines.append("\nTimber/bolt/plate checks:")
    part1_report_lines.append("Embedment strength fh,k ~= {0:.1f} N/mm2".format(fhk))
    part1_report_lines.append("Embedment group resistance ~= {0:.2f} kN; utilization = {1:.2f} [{2}]".format(Rd_embed_group, util_embed, _passfail(util_embed)))
    part1_report_lines.append("Bolt double-shear group resistance ~= {0:.2f} kN; utilization = {1:.2f} [{2}]".format(Fv_Rd_group, util_bolt_shear, _passfail(util_bolt_shear)))
    part1_report_lines.append("Steel plate bearing group resistance ~= {0:.2f} kN; utilization = {1:.2f} [{2}]".format(bearing_group, util_plate_bearing, _passfail(util_plate_bearing)))
    part1_report_lines.append("Steel plate net area = {0:.0f} mm2; net resistance ~= {1:.2f} kN; utilization = {2:.2f} [{3}]".format(A_net_plate, N_Rd_plate_net, util_plate_net, _passfail(util_plate_net)))
    part1_report_lines.append("Washer face bearing group resistance ~= {0:.2f} kN; utilization = {1:.2f} [{2}]".format(washer_capacity_group, util_washer_bearing, _passfail(util_washer_bearing)))

    part1_report_lines.append("\nTimber milling parameters:")
    part1_report_lines.append("Slot width = {0:.1f} mm for {1:.1f} mm galvanized plate".format(slot_width, t_plate))
    part1_report_lines.append("Slot length = {0:.1f} mm".format(slot_length))
    part1_report_lines.append("Slot depth/height = {0:.1f} mm".format(slot_depth))
    part1_report_lines.append("Through bolt holes = O{0:.1f} mm".format(bolt_hole_dia))
    part1_report_lines.append("Washer/nut counterbore face diameter = O{0:.1f} mm".format(washer_face_dia))
    part1_report_lines.append("Washer/nut counterbore depth = {0:.1f} mm".format(washer_recess_depth))
    part1_report_lines.append("Timber plug depth allowance = {0:.1f} mm".format(plug_depth))
    part1_report_lines.append("Remaining timber after recess = {0:.1f} mm".format(remaining_timber_after_recess))

    part1_report_lines.append("\nInstallation / access constraints, rule-of-thumb:")
    part1_report_lines.append("Clearance between converging timber members = {0:.1f} mm".format(clearance_between_converging_timbers))
    part1_report_lines.append("Preferred tool clearance = {0:.1f} mm; absolute tight-install threshold = {1:.1f} mm".format(min_tool_clearance_preferred, min_tool_clearance_absolute))
    part1_report_lines.append("Installation clearance status = {0}".format(installation_clearance_status))
    part1_report_lines.append("Minimum recess-edge clearance to obstruction = {0:.1f} mm; provided = {1:.1f} mm; OK = {2}".format(min_recess_edge_clear, adjacent_obstruction_clearance, recess_edge_clear_ok))
    part1_report_lines.append("Minimum bolt-axis-to-obstruction clearance = {0:.1f} mm; provided = {1:.1f} mm; OK = {2}".format(min_bolt_axis_to_obstruction, bolt_axis_to_obstruction, bolt_axis_clear_ok))
    part1_report_lines.append("Installation preferred OK = {0}; installation absolute OK = {1}".format(installation_preferred_ok, installation_absolute_ok))
    part1_report_lines.append("Note: If status is TIGHT, use preassembly, slim sockets, staged tightening, exposed hardware, or larger node spacing.")

    part1_report_lines.append("\nGeometry checks:")
    part1_report_lines.append("Plate depth fits timber depth: {0}".format(geometry_plate_fits_depth))
    part1_report_lines.append("Side timber thickness >= 3d: {0}".format(geometry_side_thickness_ok))
    part1_report_lines.append("Slot fits timber: {0}".format(geometry_slot_ok))
    part1_report_lines.append("Recess geometry acceptable: {0}".format(recess_geometry_ok))
    part1_report_lines.append("Installation absolute clearance acceptable: {0}".format(installation_absolute_ok))
    part1_report_lines.append("Part 1 result: {0}".format("OK PRELIMINARY" if (part1_strength_ok and geometry_ok) else "NG - REVISE GEOMETRY/SIZE/INSTALLATION ACCESS"))

    part1_report = "\n".join(part1_report_lines)

    if run_part2_weld_checks:
        weld_report, weld_utils, weld_ok = _run_part2_weld_report(
            F_Ed_part1,
            Fu_steel,
            steel_grade,
            gamma_M2,
            weld_leg_size,
            beta_w,
            weld_effective_lengths,
            weld_force_share,
            weld_eccentric_moment_Ed,
        )
    else:
        weld_report = "PART 2: WELD CHECK REPORT\nWeld checks deactivated by run_part2_weld_checks = False."
        weld_utils = []
        weld_ok = None

    if run_part3_anchor_bolt_checks:
        anchor_report, anchor_utils, anchor_ok = _run_part3_anchor_report(
            V_lat_Ed,
            N_comp_Ed,
            N_tens_Ed,
            Mx_Ed,
            My_Ed,
            gamma_M2,
            base_plate_length,
            base_plate_width,
            base_plate_thickness,
            anchor_bolt_dia,
            anchor_bolt_count,
            anchor_bolt_Fub,
            anchor_pattern_x,
            anchor_pattern_y,
            screw_pier_rated_compression,
            screw_pier_rated_tension,
            screw_pier_rated_lateral,
            screw_pier_rated_moment,
        )
    else:
        anchor_report = "PART 3: BASE PLATE TO SCREW-PIER CAP / ANCHOR BOLT CHECK REPORT\nAnchor bolt / screw-pier cap checks deactivated by run_part3_anchor_bolt_checks = False."
        anchor_utils = []
        anchor_ok = None

    combined_report = "\n\n".join([part1_report, weld_report, anchor_report])

    pass_fail_summary = {
        "part1_geometry_ok": geometry_ok,
        "part1_strength_ok": part1_strength_ok,
        "part1_installation_preferred_ok": installation_preferred_ok,
        "part1_installation_absolute_ok": installation_absolute_ok,
        "part1_installation_clearance_status": installation_clearance_status,
        "part2_weld_ok": weld_ok,
        "part3_anchor_screw_pier_ok": anchor_ok,
    }

    utilization_values = {
        "part1": part1_utils,
        "part2_weld": weld_utils,
        "part3_anchor_screw_pier": anchor_utils,
    }

    sizing_recommendations = {
        "bolt_hole_dia": bolt_hole_dia,
        "bolt_dia": bolt_dia,
        "rows": rows,
        "holes_per_row": holes_per_row,
        "pitch_parallel": pitch_parallel,
        "gage_perp": gage_perp,
        "end_distance": end_distance,
        "edge_distance": edge_distance,
        "plate_length": plate_length,
        "plate_width": plate_depth,
        "plate_thickness": t_plate,
        "anchor_pattern_x": anchor_pattern_x,
        "anchor_pattern_y": anchor_pattern_y,
        "anchor_bolt_dia": anchor_bolt_dia,
        "anchor_bolt_count": anchor_bolt_count,
    }

    fabrication_parameters = {
        "hole_clearance": hole_clearance,
        "slot_clearance_each_side": slot_clearance_each_side,
        "slot_extra_length": slot_extra_length,
        "slot_extra_depth": slot_extra_depth,
        "slot_width": slot_width,
        "slot_length": slot_length,
        "slot_depth": slot_depth,
        "washer_recess_depth": washer_recess_depth,
        "washer_face_dia": washer_face_dia,
        "plug_depth": plug_depth,
        "min_tool_clearance_preferred": min_tool_clearance_preferred,
        "min_tool_clearance_absolute": min_tool_clearance_absolute,
        "min_recess_edge_clear": min_recess_edge_clear,
        "min_bolt_axis_to_obstruction": min_bolt_axis_to_obstruction,
    }

    return {
        "a": combined_report,
        "b": timber_brep,
        "c": plate_brep,
        "d": slot_cut_brep,
        "e": bolt_breps,
        "f": hole_breps,
        "g": washer_recess_breps,
        "h": bolt_points,
        "i": utilization_values,
        "j": pass_fail_summary,
        "k": weld_report,
        "l": anchor_report,
        "m": washer_face_circles,
        "n": force_vectors,
        "combined_report": combined_report,
        "part1_report": part1_report,
        "weld_report": weld_report,
        "anchor_report": anchor_report,
        "pass_fail_summary": pass_fail_summary,
        "utilization_values": utilization_values,
        "sizing_recommendations": sizing_recommendations,
        "fabrication_parameters": fabrication_parameters,
    }


def run_validation(
    geometry_payload: Dict[str, object],
    bottom_face_mode: str = "Perpendicular_to_grain",
    min_allowable_clearance: float = 80.0,
    shift_step: float = 10.0,
    max_shift: float = 400.0,
    base_plate_min_thickness: float = 10.0,
    base_plate_max_thickness: float = 60.0,
    sync_plate_dimensions_from_engineering: bool = True,
    sync_iterations: int = 1,
    engineering_overrides: Optional[Dict[str, object]] = None,
) -> Dict[str, object]:
    cfg = ValidationConfig(
        min_allowable_clearance=min_allowable_clearance,
        shift_step=shift_step,
        max_shift=max_shift,
        base_plate_min_thickness=base_plate_min_thickness,
        base_plate_max_thickness=base_plate_max_thickness,
    )

    members = _member_map(geometry_payload)
    plates = _plate_records(geometry_payload)
    results, adjusted_plates = _run_geometric_validation_pass(members, plates, cfg)

    passed_count = sum(1 for r in results if r.passed)
    failed_count = len(results) - passed_count

    engineering = run_engineering_checks(overrides=engineering_overrides)

    sync_iters = max(1, int(sync_iterations or 1))
    sync_applied = False
    if sync_plate_dimensions_from_engineering:
        sizing = engineering.get("sizing_recommendations")
        if isinstance(sizing, dict) and sizing:
            sync_applied = True
            synced_pass_plates = adjusted_plates
            synced_pass_results = results
            for _ in range(sync_iters):
                synced_pass_plates = _apply_engineering_sizing_to_plates(synced_pass_plates, sizing)
                synced_pass_results, synced_pass_plates = _run_geometric_validation_pass(members, synced_pass_plates, cfg)
            results = synced_pass_results
            adjusted_plates = synced_pass_plates

            passed_count = sum(1 for r in results if r.passed)
            failed_count = len(results) - passed_count

    synced_geometry_payload = _build_synced_geometry_payload(
        geometry_payload=geometry_payload,
        adjusted_plates=adjusted_plates,
        sync_iterations=sync_iters if sync_applied else 0,
        sync_applied=sync_applied,
    )

    report_lines = [
        "Base Plate Validation",
        "Bottom face mode: {0}".format(bottom_face_mode),
        "Members checked: {0}".format(len(results)),
        "Passed: {0}".format(passed_count),
        "Failed: {0}".format(failed_count),
        "Min required tip-corner clearance: {0:.1f} mm".format(cfg.min_allowable_clearance),
        "Sizing sync applied: {0}".format(sync_applied),
        "Sizing sync iterations: {0}".format(sync_iters if sync_applied else 0),
    ]

    for r in results:
        report_lines.append(
            "[{0}] member={1}, group={2}, clearance={3:.1f}->{4:.1f} mm, shift={5:.1f} mm".format(
                "PASS" if r.passed else "FAIL",
                r.member_id,
                r.group,
                r.initial_min_corner_clearance,
                r.final_min_corner_clearance,
                r.adjustment_along_member,
            )
        )
        for msg in r.messages:
            report_lines.append("  - {0}".format(msg))

    payload = {
        "metadata": {
            "module": "base_plate_calculations",
            "generated_utc": datetime.utcnow().isoformat() + "Z",
            "bottom_face_mode": bottom_face_mode,
            "min_allowable_clearance": cfg.min_allowable_clearance,
            "shift_step": cfg.shift_step,
            "max_shift": cfg.max_shift,
            "sync_plate_dimensions_from_engineering": bool(sync_plate_dimensions_from_engineering),
            "sync_applied": sync_applied,
            "sync_iterations": sync_iters if sync_applied else 0,
            "passed": failed_count == 0,
        },
        "report_text": "\n".join(report_lines),
        "member_results": [
            {
                "member_id": r.member_id,
                "member_index": r.member_index,
                "group": r.group,
                "passed": r.passed,
                "initial_min_corner_clearance": r.initial_min_corner_clearance,
                "final_min_corner_clearance": r.final_min_corner_clearance,
                "adjustment_along_member": r.adjustment_along_member,
                "checks": r.checks,
                "messages": r.messages,
            }
            for r in results
        ],
        "adjusted_base_plates": adjusted_plates,
        "synced_geometry_payload": synced_geometry_payload,
        "engineering": engineering,
        "a": engineering["a"],
        "b": engineering["b"],
        "c": engineering["c"],
        "d": engineering["d"],
        "e": engineering["e"],
        "f": engineering["f"],
        "g": engineering["g"],
        "h": engineering["h"],
        "i": engineering["i"],
        "j": engineering["j"],
        "k": engineering["k"],
        "l": engineering["l"],
        "m": engineering["m"],
        "n": engineering["n"],
        "combined_report": engineering["combined_report"],
        "weld_report": engineering["weld_report"],
        "anchor_report": engineering["anchor_report"],
        "pass_fail_summary": engineering["pass_fail_summary"],
        "utilization_values": engineering["utilization_values"],
    }
    return payload


def save_validation_report(payload: Dict[str, object], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as fp:
        fp.write(str(payload.get("report_text", "")))


def _cli() -> None:
    import base_plate_geometry as geo

    g_payload = geo.build_geometry_payload()
    result = run_validation(g_payload)
    print(result["report_text"])
    print("\n---\n")
    print(result["combined_report"])


if __name__ == "__main__":
    _cli()
