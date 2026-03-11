# Living Lab HIL MAS DFAB T2 Fabrication

## Folder Structure

```
├── design/
│   ├── line_model/
│   │   └── data/          # JSON files with COMPAS line data
│   └── structure_model/
│       └── data/          # JSON files with structural analysis results
├── fabrication/
│   └── data/              # Fabrication-ready data
└── src/                   # Source code
```

---

## Workflow

### 1. Line Model (`design/line_model/`)

Contains a list of COMPAS lines with the following attributes:

| Attribute | Type | Description |
|-----------|------|-------------|
| `type` | `string` | Line type identifier |
| `z_vector` | `[x, y, z]` | Optional orientation vector |
| `cross_sections` | `list` | List of cross-section definitions |

**Output:** Stored as JSON in `design/line_model/data/out_line.json`

---

### 2. Structure Model (`design/structure/`)

1. Import line model JSON from `design/line_model/data/`
2. Load in Grasshopper (`.ghx` file)
3. Convert COMPAS lines to Rhino geometry
4. Run structural analysis with **Karamba**
5. Export updated JSON with:
   - Lines geometry
   - Updated cross-sections list (optimized from analysis)

**Output:** Stored as JSON in `design/structure/data/out_structure_lines.json`

---

### 3. Timber Design (`design/timber_design`)

1. Import COMPAS lines from structure output
2. Convert to **compas_timber** model
3. Assign joints to connections

**Output metrics:**
- Number of joints
- Number of beams
- Total timber volume
- JSON/BTLx for fabrication

---

## Dependencies

- [COMPAS](https://compas.dev/)
- [compas_timber](https://github.com/gramaziokohler/compas_timber)
- [Karamba3D](https://www.karamba3d.com/)
- Rhino/Grasshopper
