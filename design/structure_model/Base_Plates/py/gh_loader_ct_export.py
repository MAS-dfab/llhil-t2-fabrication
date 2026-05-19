"""
Tiny Grasshopper loader for the CT export wrapper.

Paste this file's contents into the GH Python component when you want the
component to execute the full on-disk wrapper without copying the long wrapper
body into RhinoCode.
"""

import traceback
from pathlib import Path

WRAPPER_PATH = Path(
    r"C:\Users\Juste\Documents\_GitHub\MAS-2526\10_llhil-t2-fabrication\design\structure_model\Base_Plates\py\gh_wrapper_ct_export.py"
)

# Fallback assignments make a loader failure visible in GH instead of leaving
# every downstream output as <null>.
out = "CT loader initialized"
package = {}
records = []
timber_model_schema = {}
inspection_breps = []
json_path = None
debug_status = {"phase": "loader_initialized"}
a = out
b = package
c = records
d = timber_model_schema
e = inspection_breps
f = json_path
g = debug_status

try:
    with WRAPPER_PATH.open("r", encoding="utf-8") as _wrapper_file:
        exec(compile(_wrapper_file.read(), str(WRAPPER_PATH), "exec"), globals())
except Exception:
    out = traceback.format_exc()
    package = {"loader_error": out}
    records = [{"loader_error": out}]
    timber_model_schema = {"loader_error": True}
    inspection_breps = []
    json_path = None
    debug_status = {"phase": "loader_error", "error": out}
    a = out
    b = package
    c = records
    d = timber_model_schema
    e = inspection_breps
    f = json_path
    g = debug_status
