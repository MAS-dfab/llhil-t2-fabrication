from __future__ import annotations

from typing import Dict, Iterable, List, Set


def _to_name_list(value: object) -> List[str]:
    """Normalize GH input (None, string, list) to a clean combo-name list."""
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


def _parse_index_spec(spec: object) -> Set[int]:
    """Parse index spec like '1-8, 12, 15-16' into a set of 1-based indices."""
    text = "" if spec is None else str(spec).strip()
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


def split_combo_names_for_design_situations(
    combo_names: object,
    uls_fundamental_indices: object = "1-8",
    sls_rare_indices: object = "9-16",
    sls_frequent_indices: object = "17-21",
    sls_qp_indices: object = "22",
    uls_accidental_fire_indices: object = "",
) -> Dict[str, List[str]]:
    """Split one combo-name list into DS inputs using 1-based index specs."""
    names = _to_name_list(combo_names)

    group_specs = {
        "ULS_Fundamental_Combos": _parse_index_spec(uls_fundamental_indices),
        "SLS_Rare_Combos": _parse_index_spec(sls_rare_indices),
        "SLS_Frequent_Combos": _parse_index_spec(sls_frequent_indices),
        "SLS_QP_Combos": _parse_index_spec(sls_qp_indices),
        "ULS_Accidental_Fire_Combos": _parse_index_spec(uls_accidental_fire_indices),
    }

    grouped: Dict[str, List[str]] = {key: [] for key in group_specs.keys()}

    for i, name in enumerate(names, start=1):
        for group_key, idx_set in group_specs.items():
            if i in idx_set:
                grouped[group_key].append(name)

    return grouped


# GH Py3 auto-run block: one ComboNames input -> 5 grouped outputs
_g = globals()
if "ComboNames" in _g:
    _grouped = split_combo_names_for_design_situations(
        combo_names=_g.get("ComboNames"),
        uls_fundamental_indices=_g.get("ULS_Fundamental_Indices", "1-8"),
        sls_rare_indices=_g.get("SLS_Rare_Indices", "9-16"),
        sls_frequent_indices=_g.get("SLS_Frequent_Indices", "17-21"),
        sls_qp_indices=_g.get("SLS_QP_Indices", "22"),
        uls_accidental_fire_indices=_g.get("ULS_Accidental_Fire_Indices", ""),
    )

    ULS_Fundamental_Combos = _grouped["ULS_Fundamental_Combos"]
    SLS_Rare_Combos = _grouped["SLS_Rare_Combos"]
    SLS_Frequent_Combos = _grouped["SLS_Frequent_Combos"]
    SLS_QP_Combos = _grouped["SLS_QP_Combos"]
    ULS_Accidental_Fire_Combos = _grouped["ULS_Accidental_Fire_Combos"]

    out = (
        "Grouped {} combos -> ULS_F: {}, SLS_R: {}, SLS_Fr: {}, SLS_QP: {}, ULS_AF: {}"
    ).format(
        len(_to_name_list(_g.get("ComboNames"))),
        len(ULS_Fundamental_Combos),
        len(SLS_Rare_Combos),
        len(SLS_Frequent_Combos),
        len(SLS_QP_Combos),
        len(ULS_Accidental_Fire_Combos),
    )


if __name__ == "__main__":
    sample = [f"ULS_C{i}" for i in range(1, 23)]
    grouped = split_combo_names_for_design_situations(sample)
    for k, v in grouped.items():
        print(k, v)
