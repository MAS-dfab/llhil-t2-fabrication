from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List


@dataclass(frozen=True)
class LimitCheckSpec:
    code: str
    name: str
    rfem_name: str
    group: str
    description: str


ULS_STANDARD_CHECKS: List[LimitCheckSpec] = [
    LimitCheckSpec("ULS_CRUSH", "Crushing / Compression Perpendicular-to-Grain", "Timber ULS - Compression Perpendicular to Grain", "Strength", "Bearing and localized compression verification."),
    LimitCheckSpec("ULS_BUCK_C", "Compression Buckling", "Timber ULS - Member Stability (Compression Buckling)", "Stability", "Column/member instability in compression."),
    LimitCheckSpec("ULS_BUCK_F", "Flexural Buckling", "Timber ULS - Member Stability (Flexural Buckling)", "Stability", "Out-of-plane member buckling under bending."),
    LimitCheckSpec("ULS_LTB", "Lateral-Torsional Buckling", "Timber ULS - Member Stability (Lateral Torsional Buckling)", "Stability", "Buckling check for bending members."),
    LimitCheckSpec("ULS_BEND_AX", "Combined Bending + Axial", "Timber ULS - Combined Stress Interaction", "Interaction", "Combined stress interaction verification."),
    LimitCheckSpec("ULS_SHEAR", "Shear Resistance", "Timber ULS - Shear", "Strength", "Shear stress and utilization check."),
]

SLS_STANDARD_CHECKS: List[LimitCheckSpec] = [
    LimitCheckSpec("SLS_DEF_INST", "Deflection (Instantaneous)", "Timber SLS - Deformation (Instantaneous)", "Serviceability", "Instantaneous deflection limit check."),
    LimitCheckSpec("SLS_DEF_FIN", "Deflection (Final)", "Timber SLS - Deformation (Final)", "Serviceability", "Final deflection including long-term effects."),
    LimitCheckSpec("SLS_SHRINK", "Timber Shrinkage", "Timber SLS - Shrinkage", "Serviceability", "Moisture-related shrinkage deformation check."),
    LimitCheckSpec("SLS_CREEP", "Creep", "Timber SLS - Creep", "Serviceability", "Long-term creep deformation amplification."),
    LimitCheckSpec("SLS_VIB", "Vibration Comfort", "Timber SLS - Vibrations", "Serviceability", "Basic vibration performance screening."),
    LimitCheckSpec("SLS_MOIST", "Moisture Movement", "Timber SLS - Moisture Movements", "Serviceability", "Moisture variation movement compatibility."),
]

FIRE_STANDARD_CHECKS: List[LimitCheckSpec] = [
    LimitCheckSpec("FIRE_CHAR", "Charring Depth / Residual Section", "Timber Fire - Charring / Residual Section", "Fire", "Section reduction due to charring."),
    LimitCheckSpec("FIRE_RES", "Fire Resistance Utilization", "Timber Fire - Resistance Verification", "Fire", "Capacity check under fire design actions."),
]


DEFAULT_DS_INDEX_SPECS: Dict[int, str] = {
    1: "1-8",
    2: "9-16",
    3: "17-21",
    4: "22",
    5: "",
}


def _to_list(value: object) -> List[str]:
    """Normalize GH input (None/string/list) into a cleaned list of strings."""
    if value is None:
        return []

    if isinstance(value, str):
        if not value.strip():
            return []
        text = value.replace("\r", "\n")
        for token in [",", ";", "\t"]:
            text = text.replace(token, "\n")
        return [part.strip() for part in text.split("\n") if part.strip()]

    if isinstance(value, Iterable):
        out: List[str] = []
        for item in value:
            if item is None:
                continue
            text = str(item).strip()
            if text:
                out.append(text)
        return out

    text = str(value).strip()
    return [text] if text else []


def _to_bool_list(value: object) -> List[bool]:
    if value is None:
        return []

    if isinstance(value, bool):
        return [value]

    if isinstance(value, str):
        tokens = _to_list(value)
    elif isinstance(value, Iterable):
        tokens = [str(item).strip() for item in value if item is not None]
    else:
        tokens = [str(value).strip()]

    true_tokens = {"1", "true", "yes", "on"}
    false_tokens = {"0", "false", "no", "off"}

    result: List[bool] = []
    for token in tokens:
        t = token.lower()
        if t in true_tokens:
            result.append(True)
        elif t in false_tokens:
            result.append(False)
        else:
            result.append(bool(token))
    return result


def _split_equation_bucket(value: str) -> List[str]:
    text = (value or "").replace("\r", "\n")
    for token in [";", "\t"]:
        text = text.replace(token, "\n")
    return [part.strip() for part in text.split("\n") if part.strip()]


def _parse_index_spec(spec: str) -> List[int]:
    text = (spec or "").strip()
    if not text:
        return []

    indices: List[int] = []
    for chunk in text.replace(";", ",").split(","):
        token = chunk.strip()
        if not token:
            continue
        if "-" in token:
            left, right = token.split("-", 1)
            try:
                start = int(left.strip())
                end = int(right.strip())
            except ValueError:
                continue
            if start <= 0 or end <= 0:
                continue
            if start <= end:
                indices.extend(range(start, end + 1))
            else:
                indices.extend(range(end, start + 1))
            continue
        try:
            idx = int(token)
        except ValueError:
            continue
        if idx > 0:
            indices.append(idx)
    return indices


def _parse_ds_order(ds_id: str) -> int | None:
    digits = "".join(ch for ch in str(ds_id) if ch.isdigit())
    if not digits:
        return None
    try:
        return int(digits)
    except ValueError:
        return None


def _rebuild_missing_ds_combo_equations(ds_ids: List[str]) -> List[str]:
    """Fallback for GH definitions that don't wire DSComboEquations.

    Reconstruct DS equation buckets from the canonical ULS_C1..ULS_C22 list.
    This keeps Karamba's load input populated even when only DS metadata is wired.
    """
    try:
        from design.structure_model.data.tools.uls_load_combination_generator import (
            as_name_and_equation_lists,
        )
    except Exception:
        try:
            from uls_load_combination_generator import as_name_and_equation_lists
        except Exception:
            return ["" for _ in ds_ids]

    try:
        _, combo_equations = as_name_and_equation_lists()
    except Exception:
        return ["" for _ in ds_ids]

    rebuilt: List[str] = []
    for ds_id in ds_ids:
        order = _parse_ds_order(ds_id)
        if order is None:
            rebuilt.append("")
            continue

        spec = DEFAULT_DS_INDEX_SPECS.get(order, "")
        idxs = _parse_index_spec(spec)
        if not idxs:
            rebuilt.append("")
            continue

        selected = [combo_equations[i - 1] for i in idxs if 0 < i <= len(combo_equations)]
        rebuilt.append("; ".join(selected) if selected else "")

    return rebuilt


def _norm(text: str) -> str:
    return "".join(ch.lower() for ch in text if ch.isalnum())


def _resolve_profile_name(category: str, profile_hint: str | None) -> str:
    """Resolve profile using DS category and optional value-list token."""
    c = _norm(category)
    p = _norm(profile_hint or "")

    if "fire" in p:
        return "FIRE_STANDARD"
    if "serviceability" in p or p.startswith("sls"):
        return "SLS_STANDARD"
    if "ultimate" in p or p.startswith("uls"):
        return "ULS_STANDARD"

    if c.startswith("sls"):
        return "SLS_STANDARD"
    if "fire" in c:
        return "FIRE_STANDARD"
    return "ULS_STANDARD"


def _checks_for_profile(profile_name: str) -> List[LimitCheckSpec]:
    if profile_name == "SLS_STANDARD":
        return SLS_STANDARD_CHECKS
    if profile_name == "FIRE_STANDARD":
        return FIRE_STANDARD_CHECKS
    return ULS_STANDARD_CHECKS


def generate_design_limit_checks(
    ds_ids: object = None,
    ds_categories: object = None,
    ds_names: object = None,
    ds_combo_equations: object = None,
    configuration_profiles: object = None,
    check_prefix: str = "CHK",
) -> List[Dict[str, str]]:
    """Expand design situations into timber design-limit checks.

    Inputs expected from GH:
    - ds_ids: list like ["DS1", "DS2", ...]
    - ds_categories: list like ["ULS", "SLS", ...]
    - ds_names: optional labels
    - configuration_profiles: optional value-list tokens per DS
      Example tokens:
      - "Ultimate Configurations - 1 - Standard"
      - "Serviceability Configurations - 1 - Standard"
      - "Fire Resistance Configurations - 1 - Standard"
    """
    ids = _to_list(ds_ids)
    categories = _to_list(ds_categories)
    names = _to_list(ds_names)
    combo_equations = _to_list(ds_combo_equations)
    profiles = _to_list(configuration_profiles)

    n = len(ids)
    if len(categories) < n:
        categories.extend(["ULS"] * (n - len(categories)))
    if len(names) < n:
        names.extend([""] * (n - len(names)))
    if len(combo_equations) < n:
        combo_equations.extend([""] * (n - len(combo_equations)))
    if len(profiles) < n:
        profiles.extend([""] * (n - len(profiles)))

    # Fallback path for GH definitions where DSComboEquations is not wired.
    if n > 0 and not any((eq or "").strip() for eq in combo_equations):
        combo_equations = _rebuild_missing_ds_combo_equations(ids)

    records: List[Dict[str, str]] = []
    for i in range(n):
        ds_id = ids[i]
        ds_category = categories[i]
        ds_name = names[i] if i < len(names) else ""
        ds_loads = combo_equations[i] if i < len(combo_equations) else ""
        profile_name = _resolve_profile_name(ds_category, profiles[i] if i < len(profiles) else "")
        checks = _checks_for_profile(profile_name)

        for check_idx, check in enumerate(checks, start=1):
            records.append(
                {
                    "check_id": f"{check_prefix}_{ds_id}_{check_idx:02d}",
                    "design_situation_id": ds_id,
                    "design_situation_name": ds_name,
                    "design_situation_category": ds_category,
                    "design_situation_loads": ds_loads,
                    "profile": profile_name,
                    "check_code": check.code,
                    "check_name": check.name,
                    "rfem_check_name": check.rfem_name,
                    "check_group": check.group,
                    "check_description": check.description,
                }
            )
    return records


def as_output_lists(
    ds_ids: object = None,
    ds_categories: object = None,
    ds_names: object = None,
    ds_combo_equations: object = None,
    configuration_profiles: object = None,
    active_check_ids: object = None,
    active_check_codes: object = None,
    active_toggles: object = None,
    selected_check: object = None,
    selected_check_toggle: object = None,
    check_prefix: str = "CHK",
) -> Dict[str, List[str]]:
    """Parallel GH outputs for design limit checks."""
    rows = generate_design_limit_checks(
        ds_ids=ds_ids,
        ds_categories=ds_categories,
        ds_names=ds_names,
        ds_combo_equations=ds_combo_equations,
        configuration_profiles=configuration_profiles,
        check_prefix=check_prefix,
    )

    selected = _to_list(selected_check)
    selected_toggle_flags = _to_bool_list(selected_check_toggle)
    selected_enabled = selected_toggle_flags[0] if selected_toggle_flags else True

    selected_ids = selected if selected_enabled else []
    active_ids = set(_to_list(active_check_ids) + selected_ids)
    active_codes = set(_to_list(active_check_codes))
    toggles = _to_bool_list(active_toggles)

    loads_out: List[str] = []
    for idx, row in enumerate(rows):
        is_selected = False

        if active_ids and row["check_id"] in active_ids:
            is_selected = True
        if active_codes and row["check_code"] in active_codes:
            is_selected = True
        if toggles and idx < len(toggles) and toggles[idx]:
            is_selected = True

        if not active_ids and not active_codes and not toggles and not selected_ids:
            # Default behavior when no toggle/selection is wired: pass all loads through.
            is_selected = True

        if is_selected:
            loads_out.extend(_split_equation_bucket(str(row.get("design_situation_loads", ""))))

    # Preserve order while removing duplicates.
    seen = set()
    loads_unique: List[str] = []
    for load in loads_out:
        if load in seen:
            continue
        seen.add(load)
        loads_unique.append(load)

    return {
        "CheckIds": [str(r["check_id"]) for r in rows],
        "ParentDSIds": [str(r["design_situation_id"]) for r in rows],
        "ParentDSNames": [str(r["design_situation_name"]) for r in rows],
        "ParentDSCategories": [str(r["design_situation_category"]) for r in rows],
        "ParentDSLoads": [str(r["design_situation_loads"]) for r in rows],
        "Profiles": [str(r["profile"]) for r in rows],
        "CheckCodes": [str(r["check_code"]) for r in rows],
        "CheckNames": [str(r["check_name"]) for r in rows],
        "RFEMCheckNames": [str(r["rfem_check_name"]) for r in rows],
        "CheckGroups": [str(r["check_group"]) for r in rows],
        "CheckDescriptions": [str(r["check_description"]) for r in rows],
        "Loads": loads_unique,
    }


def as_rfem_records(
    ds_ids: object = None,
    ds_categories: object = None,
    ds_names: object = None,
    ds_combo_equations: object = None,
    configuration_profiles: object = None,
    check_prefix: str = "CHK",
) -> List[Dict[str, str]]:
    """RFEM-friendly records for limit checks."""
    return generate_design_limit_checks(
        ds_ids=ds_ids,
        ds_categories=ds_categories,
        ds_names=ds_names,
        ds_combo_equations=ds_combo_equations,
        configuration_profiles=configuration_profiles,
        check_prefix=check_prefix,
    )


# GH Py3 auto-run block.
_g = globals()
if "DSIds" in _g:
    _lists = as_output_lists(
        ds_ids=_g.get("DSIds"),
        ds_categories=_g.get("DSCategories"),
        ds_names=_g.get("DSNames"),
        ds_combo_equations=_g.get("DSComboEquations"),
        configuration_profiles=_g.get("LimitCheckProfiles"),
        active_check_ids=_g.get("ActiveCheckIds"),
        active_check_codes=_g.get("ActiveCheckCodes"),
        active_toggles=_g.get("ActiveToggles"),
        selected_check=_g.get("SelectedCheck"),
        selected_check_toggle=_g.get("SelectedCheckToggle"),
        check_prefix=str(_g.get("CheckPrefix", "CHK")),
    )

    CheckIds = _lists["CheckIds"]
    ParentDSIds = _lists["ParentDSIds"]
    ParentDSNames = _lists["ParentDSNames"]
    ParentDSCategories = _lists["ParentDSCategories"]
    ParentDSLoads = _lists["ParentDSLoads"]
    Profiles = _lists["Profiles"]
    CheckCodes = _lists["CheckCodes"]
    CheckNames = _lists["CheckNames"]
    RFEMCheckNames = _lists["RFEMCheckNames"]
    CheckGroups = _lists["CheckGroups"]
    CheckDescriptions = _lists["CheckDescriptions"]
    Loads = _lists["Loads"]

    out = "Generated {} limit checks".format(len(CheckIds))


if __name__ == "__main__":
    sample = as_output_lists(
        ds_ids=["DS1", "DS2", "DS3", "DS4", "DS5"],
        ds_categories=["ULS", "SLS", "SLS", "SLS", "ULS_FIRE"],
        ds_names=[
            "ULS Type 2 - Fundamental",
            "SLS - Rare",
            "SLS - Frequent",
            "SLS - Quasi-permanent",
            "ULS Type 2 - Accidental - Fire",
        ],
        ds_combo_equations=[
            "1.35 * DL_T + 1.35 * DL_FD + 1.35 * LL",
            "1.60 * DL_T + 1.60 * DL_FD + 1.60 * LL",
            "1.60 * DL_T + 1.60 * DL_FD + 1.60 * LL + 0.94 * SL",
            "1.00 * DL_T + 1.00 * DL_FD + 1.00 * LL",
            "1.00 * DL_T + 1.00 * DL_FD + 1.00 * LL + FIRE",
        ],
        configuration_profiles=[
            "Ultimate Configurations - 1 - Standard",
            "Serviceability Configurations - 1 - Standard",
            "Serviceability Configurations - 1 - Standard",
            "Serviceability Configurations - 1 - Standard",
            "Fire Resistance Configurations - 1 - Standard",
        ],
    )
    print("checks:", len(sample["CheckIds"]))
