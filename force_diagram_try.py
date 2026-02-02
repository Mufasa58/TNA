"""
force_diagram_try.py
RunnerV2-compatible: run() with no args.

Loads the rebuilt FORM json produced by fan_vault_pipeline.py and tries
to create a force diagram from it.

If it fails, it returns traceback (doesn't crash your workflow).
"""

import os
import traceback
from compas.data import json_load, json_dump
from compas_tna.diagrams import ForceDiagram

# EDIT THIS to point to the output of the pipeline:
FORM_JSON = r"C:\Users\musta\Desktop\RhinoVAULT_Fan_Vault_V2_3_FORM_REBUILT.json"
OUT_FORCE = ""  # "" => same dir as FORM_JSON

def run():
    try:
        if not os.path.isfile(FORM_JSON):
            raise FileNotFoundError(FORM_JSON)

        form = json_load(FORM_JSON)

        # Fresh build; avoids update_force_from_form ordering.
        force = ForceDiagram.from_formdiagram(form)

        out_dir = OUT_FORCE or os.path.dirname(FORM_JSON)
        base = os.path.splitext(os.path.basename(FORM_JSON))[0].replace("_FORM_REBUILT", "")
        out_path = os.path.join(out_dir, base + "_FORCE.json")
        json_dump(force, out_path)

        return {"ok": True, "force_file": out_path, "force_VEF": (force.number_of_vertices(), force.number_of_edges(), force.number_of_faces())}
    except Exception:
        return {"ok": False, "error": traceback.format_exc()}
