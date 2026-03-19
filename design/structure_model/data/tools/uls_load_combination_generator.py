from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Tuple


TermMap = Dict[str, float]


@dataclass
class LoadFactors:
    # Dead + live factors
    DLF: float = 1.35
    LLF: float = 1.50
    LLF_rf: float = 1.05
    CLF: float = 1.60

    # Wind factors
    WLF_0: float = 1.05
    WLF_1: float = 1.50
    WLF_2: float = 1.00
    WLF_3: float = 0.94
    WLF_4: float = 0.60
    WLF_5: float = 0.50

    # Snow factors
    SLF_1: float = 0.94
    SLF_2: float = 1.50
    SLF_3: float = 1.00
    SLF_4: float = 0.75


@dataclass
class LoadDesignators:
    DL_T: str = "DL_T"
    DL_FD: str = "DL_FD"
    LL: str = "LL"
    WL_X: str = "WL_X"
    WL_Y: str = "WL_Y"
    SL: str = "SL"


def _to_string_list(value: object) -> List[str]:
    if value is None:
        return []

    if isinstance(value, str):
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


def _resolve_designators_from_loads(loads: object, base: LoadDesignators) -> LoadDesignators:
    """Allow GH list/dict load input to override default load-case designators.

    Supported formats:
    - Dict with keys: DL_T, DL_FD, LL, WL_X, WL_Y, SL
    - List in order: [DL_T, DL_FD, LL, WL_X, WL_Y, SL]
    - List with key-value tokens: ["DL_T=LC1", "WL_X=LC4", ...]
    """
    if loads is None:
        return base

    default_map = {
        "DL_T": base.DL_T,
        "DL_FD": base.DL_FD,
        "LL": base.LL,
        "WL_X": base.WL_X,
        "WL_Y": base.WL_Y,
        "SL": base.SL,
    }

    if isinstance(loads, dict):
        for key in default_map.keys():
            value = loads.get(key)
            if value is not None and str(value).strip():
                default_map[key] = str(value).strip()
        return LoadDesignators(**default_map)

    tokens = _to_string_list(loads)
    if not tokens:
        return base

    has_key_value = any("=" in token or ":" in token for token in tokens)
    if has_key_value:
        for token in tokens:
            if "=" in token:
                raw_key, raw_value = token.split("=", 1)
            elif ":" in token:
                raw_key, raw_value = token.split(":", 1)
            else:
                continue
            key = raw_key.strip().upper()
            value = raw_value.strip()
            if key in default_map and value:
                default_map[key] = value
        return LoadDesignators(**default_map)

    ordered_keys = ["DL_T", "DL_FD", "LL", "WL_X", "WL_Y", "SL"]
    for i, token in enumerate(tokens[: len(ordered_keys)]):
        if token:
            default_map[ordered_keys[i]] = token
    return LoadDesignators(**default_map)


def _merge_terms(*term_maps: TermMap) -> TermMap:
    merged: TermMap = {}
    for term_map in term_maps:
        for load_case, value in term_map.items():
            merged[load_case] = merged.get(load_case, 0.0) + float(value)
    return merged


def _format_expression(terms: TermMap) -> str:
    def lc_sort_key(item: Tuple[str, float]) -> int:
        name = item[0]
        digits = "".join(ch for ch in name if ch.isdigit())
        return int(digits) if digits else 10**9

    ordered = sorted(terms.items(), key=lc_sort_key)
    parts = [f"{coeff:.2f} * {load_case}" for load_case, coeff in ordered if abs(coeff) > 1e-12]
    return " + ".join(parts)


def generate_uls_combinations(
    factors: LoadFactors | None = None,
    designators: LoadDesignators | None = None,
    loads: object = None,
    combo_prefix: str = "ULS_C",
) -> List[Dict[str, object]]:
    """Generate load combinations named ULS_C1..ULS_C22.

    The generated pattern follows the shared ULS families with two wind directions.

    Args:
        factors: Numeric factors (defaults match your screenshot values).
        designators: Optional load designators for dead/live/wind/snow labels.
        loads: Optional load designator input (list/dict) from GH.
        combo_prefix: Prefix for the combination names, default "ULS_C".

    Returns:
        List of dicts with keys: name, terms, expression.
    """
    f = factors or LoadFactors()
    base_designators = designators or LoadDesignators()
    d = _resolve_designators_from_loads(loads, base_designators)

    base_uls = {
        d.DL_T: f.DLF,
        d.DL_FD: f.DLF,
        d.LL: f.DLF,
    }
    base_sr = {
        d.DL_T: f.CLF,
        d.DL_FD: f.CLF,
        d.LL: f.CLF,
    }
    winds = [d.WL_X, d.WL_Y]

    combinations: List[TermMap] = []

    # C1..C8 family
    combinations.append(_merge_terms(base_uls))
    for wind in winds:
        combinations.append(_merge_terms(base_uls, {wind: f.WLF_1}))
    for wind in winds:
        combinations.append(_merge_terms(base_uls, {wind: f.WLF_1, d.SL: f.WLF_3}))
    combinations.append(_merge_terms(base_uls, {d.SL: f.WLF_1}))
    for wind in winds:
        combinations.append(_merge_terms(base_uls, {wind: f.WLF_4, d.SL: f.WLF_1}))

    # C9..C16 family
    combinations.append(_merge_terms(base_sr))
    for wind in winds:
        combinations.append(_merge_terms(base_sr, {wind: f.WLF_2}))
    for wind in winds:
        combinations.append(_merge_terms(base_sr, {wind: f.WLF_2, d.SL: f.SLF_1}))
    combinations.append(_merge_terms(base_sr, {d.SL: f.SLF_3}))
    for wind in winds:
        combinations.append(_merge_terms(base_sr, {wind: f.WLF_4, d.SL: f.SLF_3}))

    # C17..C21 family
    combinations.append(_merge_terms(base_sr))
    for wind in winds:
        combinations.append(_merge_terms(base_sr, {wind: f.WLF_5}))
    combinations.append(_merge_terms(base_sr, {d.SL: f.SLF_4}))
    combinations.append(_merge_terms(base_sr))

    # C22 fallback/base
    combinations.append({d.DL_T: 1.0, d.DL_FD: 1.0, d.LL: 1.0})

    result: List[Dict[str, object]] = []
    for idx, terms in enumerate(combinations, start=1):
        result.append(
            {
                "name": f"{combo_prefix}{idx}",
                "terms": terms,
                "expression": _format_expression(terms),
            }
        )

    return result


def as_expression_list(
    factors: LoadFactors | None = None,
    designators: LoadDesignators | None = None,
    loads: object = None,
    combo_prefix: str = "ULS_C",
) -> List[str]:
    """Convenience output: ['ULS_C1 = ...', 'ULS_C2 = ...', ...]."""
    combos = generate_uls_combinations(
        factors=factors,
        designators=designators,
        loads=loads,
        combo_prefix=combo_prefix,
    )
    return [f"{combo['name']} = {combo['expression']}" for combo in combos]


def as_rfem_rows(
    factors: LoadFactors | None = None,
    designators: LoadDesignators | None = None,
    loads: object = None,
    combo_prefix: str = "ULS_C",
) -> List[List[str]]:
    """RFEM-friendly rows: [[name, expression], ...]."""
    combos = generate_uls_combinations(
        factors=factors,
        designators=designators,
        loads=loads,
        combo_prefix=combo_prefix,
    )
    return [[str(combo["name"]), str(combo["expression"])] for combo in combos]


def as_rfem_records(
    factors: LoadFactors | None = None,
    designators: LoadDesignators | None = None,
    loads: object = None,
    combo_prefix: str = "ULS_C",
) -> List[Dict[str, str]]:
    """RFEM-friendly records: [{"name": ..., "equation": ...}, ...]."""
    combos = generate_uls_combinations(
        factors=factors,
        designators=designators,
        loads=loads,
        combo_prefix=combo_prefix,
    )
    return [
        {
            "name": str(combo["name"]),
            "equation": str(combo["expression"]),
        }
        for combo in combos
    ]


def as_name_and_equation_lists(
    factors: LoadFactors | None = None,
    designators: LoadDesignators | None = None,
    loads: object = None,
    combo_prefix: str = "ULS_C",
) -> Tuple[List[str], List[str]]:
    """Return two parallel outputs: ComboNames and ComboEquations."""
    combos = generate_uls_combinations(
        factors=factors,
        designators=designators,
        loads=loads,
        combo_prefix=combo_prefix,
    )
    combo_names = [str(combo["name"]) for combo in combos]
    combo_equations = [str(combo["expression"]) for combo in combos]
    return combo_names, combo_equations


def factors_from_slider_inputs(
    DLF: float | None = 1.35,
    LLF: float | None = 1.50,
    LLF_rf: float | None = 1.05,
    CLF: float | None = 1.60,
    WLF_0: float | None = 1.05,
    WLF_1: float | None = 1.50,
    WLF_2: float | None = 1.00,
    WLF_3: float | None = 0.94,
    WLF_4: float | None = 0.60,
    WLF_5: float | None = 0.50,
    SLF_1: float | None = 0.94,
    SLF_2: float | None = 1.50,
    SLF_3: float | None = 1.00,
    SLF_4: float | None = 0.75,
) -> LoadFactors:
    """Build LoadFactors from GH slider values."""
    def _num(value: float | None, fallback: float) -> float:
        return fallback if value is None else float(value)

    return LoadFactors(
        DLF=_num(DLF, 1.35),
        LLF=_num(LLF, 1.50),
        LLF_rf=_num(LLF_rf, 1.05),
        CLF=_num(CLF, 1.60),
        WLF_0=_num(WLF_0, 1.05),
        WLF_1=_num(WLF_1, 1.50),
        WLF_2=_num(WLF_2, 1.00),
        WLF_3=_num(WLF_3, 0.94),
        WLF_4=_num(WLF_4, 0.60),
        WLF_5=_num(WLF_5, 0.50),
        SLF_1=_num(SLF_1, 0.94),
        SLF_2=_num(SLF_2, 1.50),
        SLF_3=_num(SLF_3, 1.00),
        SLF_4=_num(SLF_4, 0.75),
    )


def slider_inputs_to_name_and_equation_lists(
    DLF: float | None = 1.35,
    LLF: float | None = 1.50,
    LLF_rf: float | None = 1.05,
    CLF: float | None = 1.60,
    WLF_0: float | None = 1.05,
    WLF_1: float | None = 1.50,
    WLF_2: float | None = 1.00,
    WLF_3: float | None = 0.94,
    WLF_4: float | None = 0.60,
    WLF_5: float | None = 0.50,
    SLF_1: float | None = 0.94,
    SLF_2: float | None = 1.50,
    SLF_3: float | None = 1.00,
    SLF_4: float | None = 0.75,
    loads: object = None,
    designators: LoadDesignators | None = None,
    combo_prefix: str = "ULS_C",
) -> Tuple[List[str], List[str]]:
    """One-call GH adapter: slider inputs -> (ComboNames, ComboEquations)."""
    factors = factors_from_slider_inputs(
        DLF=DLF,
        LLF=LLF,
        LLF_rf=LLF_rf,
        CLF=CLF,
        WLF_0=WLF_0,
        WLF_1=WLF_1,
        WLF_2=WLF_2,
        WLF_3=WLF_3,
        WLF_4=WLF_4,
        WLF_5=WLF_5,
        SLF_1=SLF_1,
        SLF_2=SLF_2,
        SLF_3=SLF_3,
        SLF_4=SLF_4,
    )
    return as_name_and_equation_lists(
        factors=factors,
        designators=designators,
        loads=loads,
        combo_prefix=combo_prefix,
    )


# If this code is pasted directly into a GH Py3 component, compute outputs immediately.
try:
    _g = globals()
    ComboNames, ComboEquations = slider_inputs_to_name_and_equation_lists(
        DLF=_g.get("DLF"),
        LLF=_g.get("LLF"),
        LLF_rf=_g.get("LLF_rf"),
        CLF=_g.get("CLF"),
        WLF_0=_g.get("WLF_0"),
        WLF_1=_g.get("WLF_1"),
        WLF_2=_g.get("WLF_2"),
        WLF_3=_g.get("WLF_3"),
        WLF_4=_g.get("WLF_4"),
        WLF_5=_g.get("WLF_5"),
        SLF_1=_g.get("SLF_1"),
        SLF_2=_g.get("SLF_2"),
        SLF_3=_g.get("SLF_3"),
        SLF_4=_g.get("SLF_4"),
        loads=_g.get("Loads"),
    )
except NameError:
    # Not running inside GH component inputs; keep module import-safe.
    pass


if __name__ == "__main__":
    for line in as_expression_list():
        print(line)
