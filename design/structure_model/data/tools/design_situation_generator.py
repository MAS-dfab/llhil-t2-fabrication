from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Set


@dataclass
class DesignSituationSpec:
    order: int
    short_code: str
    name: str
    category: str
    index_spec: str


DEFAULT_DESIGN_SITUATIONS: List[DesignSituationSpec] = [
    DesignSituationSpec(order=1, short_code="2 F", name="ULS Type 2 - Fundamental", category="ULS", index_spec="1-8"),
    DesignSituationSpec(order=2, short_code="S R", name="SLS - Rare", category="SLS", index_spec="9-16"),
    DesignSituationSpec(order=3, short_code="S Fr", name="SLS - Frequent", category="SLS", index_spec="17-21"),
    DesignSituationSpec(order=4, short_code="S Qp", name="SLS - Quasi-permanent", category="SLS", index_spec="22"),
    DesignSituationSpec(order=5, short_code="2 AF", name="ULS Type 2 - Accidental - Fire - psi 2,1", category="ULS", index_spec=""),
]


def _to_combo_list(value: object) -> List[str]:
    """Normalize GH input (None, string, list) into a clean combo-name list."""
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
        result: List[str] = []
        for item in value:
            if item is None:
                continue
            text = str(item).strip()
            if text:
                result.append(text)
        return result

    text = str(value).strip()
    return [text] if text else []


def _parse_index_spec(spec: str) -> Set[int]:
    """Parse 1-based index ranges like '1-8, 12, 15-16'."""
    text = (spec or "").strip()
    if not text:
        return set()

    result: Set[int] = set()
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
                result.update(range(start, end + 1))
            else:
                result.update(range(end, start + 1))
            continue

        try:
            idx = int(token)
        except ValueError:
            continue
        if idx > 0:
            result.add(idx)

    return result


def _select_combo_names(combo_names: List[str], index_spec: str) -> List[str]:
    indices = _parse_index_spec(index_spec)
    if not indices:
        return []
    return [name for i, name in enumerate(combo_names, start=1) if i in indices]


def _select_combo_equations(combo_equations: List[str], index_spec: str) -> List[str]:
    indices = _parse_index_spec(index_spec)
    if not indices:
        return []
    return [equation for i, equation in enumerate(combo_equations, start=1) if i in indices]


def _normalize_token(text: str) -> str:
    return "".join(ch.lower() for ch in text if ch.isalnum())


def _find_default_spec(token: str) -> DesignSituationSpec | None:
    t = _normalize_token(token)
    if not t:
        return None

    alias_by_order = {
        1: ["ULS_Fundamental_Combos"],
        2: ["SLS_Rare_Combos"],
        3: ["SLS_Frequent_Combos"],
        4: ["SLS_QP_Combos"],
        5: ["ULS_Accidental_Fire_Combos"],
    }

    for spec in DEFAULT_DESIGN_SITUATIONS:
        candidates = [
            f"ds{spec.order}",
            spec.short_code,
            spec.name,
            f"DS{spec.order} - {spec.name}",
            f"{spec.short_code} {spec.name}",
        ]
        candidates.extend(alias_by_order.get(spec.order, []))
        if any(_normalize_token(candidate) == t for candidate in candidates):
            return spec
    return None


def _resolve_design_situations(design_situations: object) -> List[DesignSituationSpec]:
    """Resolve value-list input into DS specs.

    Supported item formats:
    - "DS1" / "DS2" / ...
    - "DS1 - ULS Type 2 - Fundamental"
    - "2 F ULS Type 2 - Fundamental"
    - Custom pipe format: "DS6|X|Custom Name|ULS|1-3,7"
    """
    raw_items = _to_combo_list(design_situations)
    if not raw_items:
        return list(DEFAULT_DESIGN_SITUATIONS)

    resolved: List[DesignSituationSpec] = []
    for idx, item in enumerate(raw_items, start=1):
        text = item.strip()
        if not text:
            continue

        if "|" in text:
            parts = [p.strip() for p in text.split("|")]
            if len(parts) >= 5:
                order_token, short_code, name, category, index_spec = parts[:5]
                digits = "".join(ch for ch in order_token if ch.isdigit())
                order = int(digits) if digits else idx
                resolved.append(
                    DesignSituationSpec(
                        order=order,
                        short_code=short_code or "-",
                        name=name or f"Custom DS {order}",
                        category=(category or "ULS").upper(),
                        index_spec=index_spec,
                    )
                )
                continue

        default_spec = _find_default_spec(text)
        if default_spec is not None:
            resolved.append(default_spec)

    return resolved if resolved else list(DEFAULT_DESIGN_SITUATIONS)


def generate_design_situations(
    combo_names: object = None,
    combo_equations: object = None,
    design_situations: object = None,
    ds_prefix: str = "DS",
) -> List[Dict[str, object]]:
    """Build design situations from two inputs: ComboNames + DesignSituations."""
    combo_list = _to_combo_list(combo_names)
    combo_eq_list = _to_combo_list(combo_equations)
    specs = _resolve_design_situations(design_situations)

    records: List[Dict[str, object]] = []
    for spec in specs:
        combos = _select_combo_names(combo_list, spec.index_spec)
        equations = _select_combo_equations(combo_eq_list, spec.index_spec)
        records.append(
            {
                "id": f"{ds_prefix}{spec.order}",
                "short_code": spec.short_code,
                "name": spec.name,
                "category": spec.category,
                "index_spec": spec.index_spec,
                "combos": combos,
                "combo_equations": equations,
            }
        )
    return records


def as_rfem_records(
    combo_names: object = None,
    combo_equations: object = None,
    design_situations: object = None,
    ds_prefix: str = "DS",
) -> List[Dict[str, str]]:
    """RFEM-friendly DS records with joined combination names."""
    records = generate_design_situations(
        combo_names=combo_names,
        combo_equations=combo_equations,
        design_situations=design_situations,
        ds_prefix=ds_prefix,
    )

    out: List[Dict[str, str]] = []
    for row in records:
        combos = row["combos"]
        equations = row["combo_equations"]
        combo_text = "; ".join(str(x) for x in combos) if combos else ""
        equation_text = "; ".join(str(x) for x in equations) if equations else ""
        out.append(
            {
                "id": str(row["id"]),
                "short_code": str(row["short_code"]),
                "name": str(row["name"]),
                "category": str(row["category"]),
                "index_spec": str(row["index_spec"]),
                "combination_names": combo_text,
                "combination_equations": equation_text,
            }
        )
    return out


def as_output_lists(
    combo_names: object = None,
    combo_equations: object = None,
    design_situations: object = None,
    ds_prefix: str = "DS",
) -> Dict[str, List[str]]:
    """Parallel lists for GH outputs."""
    records = generate_design_situations(
        combo_names=combo_names,
        combo_equations=combo_equations,
        design_situations=design_situations,
        ds_prefix=ds_prefix,
    )

    ds_ids = [str(r["id"]) for r in records]
    ds_short_codes = [str(r["short_code"]) for r in records]
    ds_names = [str(r["name"]) for r in records]
    ds_categories = [str(r["category"]) for r in records]
    ds_index_specs = [str(r["index_spec"]) for r in records]
    ds_combo_lists = [list(r["combos"]) for r in records]
    ds_combo_eq_lists = [list(r["combo_equations"]) for r in records]

    return {
        "DSIds": ds_ids,
        "DSShortCodes": ds_short_codes,
        "DSNames": ds_names,
        "DSCategories": ds_categories,
        "DSIndexSpecs": ds_index_specs,
        "DSComboNames": ["; ".join(items) for items in ds_combo_lists],
        "DSComboEquations": ["; ".join(items) for items in ds_combo_eq_lists],
    }


# GH Py3 auto-run block: two input ports expected -> ComboNames + DesignSituations.
_g = globals()
if "ComboNames" in _g:
    _lists = as_output_lists(
        combo_names=_g.get("ComboNames"),
        combo_equations=_g.get("ComboEquations"),
        design_situations=_g.get("DesignSituations"),
        ds_prefix=str(_g.get("DSPrefix", "DS")),
    )
    DSIds = _lists["DSIds"]
    DSShortCodes = _lists["DSShortCodes"]
    DSNames = _lists["DSNames"]
    DSCategories = _lists["DSCategories"]
    DSIndexSpecs = _lists["DSIndexSpecs"]
    DSComboNames = _lists["DSComboNames"]
    DSComboEquations = _lists["DSComboEquations"]
    out = "Generated {} design situations".format(len(DSIds))


if __name__ == "__main__":
    sample = as_rfem_records(
        combo_names=[f"ULS_C{i}" for i in range(1, 23)],
        combo_equations=[f"1.35 * LC{i}" for i in range(1, 23)],
        design_situations=["DS1", "DS2", "DS3", "DS4", "DS5"],
    )
    for row in sample:
        print(row)
