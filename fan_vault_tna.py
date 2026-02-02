"""
fan_vault_pipeline.py
RunnerV2-compatible: exposes run() with no args.

Pipeline:
1) Load RhinoVAULT session JSON (raw json) from disk
2) Extract Pattern (dtype: compas_rv.datastructures/Pattern)
3) Build FormDiagram from Pattern (clean, stable)
4) Build ThrustDiagram from FormDiagram
5) Solve horizontal + vertical (basic)
6) (Optional) build intrados/extrados as offset meshes (COMPAS Mesh copies)
7) Dump outputs to JSON next to input file (clean form, thrust, intrados, extrados)

Notes:
- This avoids the buggy RV "update_force_from_form" path that threw KeyError: 47.
- Works with RV session exports that include Pattern (your file does). The file also includes TNA settings. :contentReference[oaicite:2]{index=2}
"""

from logging import root
import os
import json
import traceback

from compas.data import DataDecoder, json_dump

from compas_rv.datastructures import Pattern, FormDiagram, ThrustDiagram
from compas_tna.equilibrium import horizontal_nodal, vertical_from_zmax

from compas.geometry import add_vectors, scale_vector, normalize_vector


# ==========================
# CONFIG (EDIT THESE)
# ==========================
SESSION_JSON = r'/Users/mmg/dev/tna_playground/JSON Files/RhinoVAULT_Fan_Vault_V2_3.json' # <-- PUT YOUR FILE PATH HERE
OUT_DIR = ""  # "" = same folder as SESSION_JSON, or set explicit folder

# Solve params (defaults match your session settings closely) :contentReference[oaicite:3]{index=3}
H_KMAX = 100
H_ALPHA = 100.0
V_KMAX = 300
ZMAX = None  # None = read from session settings if available, else fallback

# Intrados / extrados
MAKE_INTRA_EXTRA = True
THICKNESS = 0.12  # model units (meters if your model is in meters)

# Cleaning
DELETE_NON_EDGES = True
DELETE_ISOLATED_VERTICES = True


# ==========================
# INTERNAL HELPERS
# ==========================
def walk_nodes(node):
    if isinstance(node, dict):
        yield node
        for v in node.values():
            yield from walk_nodes(v)
    elif isinstance(node, list):
        for v in node:
            yield from walk_nodes(v)

def find_first_dtype(root, dtype_exact):
    for n in walk_nodes(root):
        if n.get("dtype") == dtype_exact and "data" in n:
            return n
    return None

def load_raw_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def read_zmax_from_session(session_dict):
    # RV session stores it under: settings.tna.vertical_zmax :contentReference[oaicite:4]{index=4}
    try:
        return float(session_dict["scene"]["data"]["settings"]["tna"]["vertical_zmax"])
    except Exception:
        return None

def delete_non_edges(diagram):
    # RV sometimes stores edges with "_is_edge": false (these break ordered edge assumptions) :contentReference[oaicite:5]{index=5}
    to_delete = []
    for u, v in list(diagram.edges()):
        if diagram.edge_attribute((u, v), "_is_edge") is False:
            to_delete.append((u, v))
    for u, v in to_delete:
        if diagram.has_edge(u, v):
            diagram.delete_edge(u, v)
    return len(to_delete)

def delete_isolated_vertices(diagram):
    isolated = [v for v in diagram.vertices() if diagram.vertex_degree(v) == 0]
    for v in isolated:
        diagram.delete_vertex(v)
    return len(isolated)

def mesh_vertex_normals(mesh):
    # average adjacent face normals
    vnormals = {v: [0.0, 0.0, 0.0] for v in mesh.vertices()}
    for f in mesh.faces():
        n = mesh.face_normal(f)
        for v in mesh.face_vertices(f):
            vnormals[v] = add_vectors(vnormals[v], n)
    for v in vnormals:
        vnormals[v] = normalize_vector(vnormals[v])
    return vnormals

def offset_mesh_along_normals(mesh, offset):
    out = mesh.copy()
    VN = mesh_vertex_normals(mesh)
    for v in out.vertices():
        p = out.vertex_coordinates(v)
        n = VN[v]
        out.vertex_attributes(v, "xyz", add_vectors(p, scale_vector(n, offset)))
    return out

def out_path(base_name, out_dir, stem):
    os.makedirs(out_dir, exist_ok=True)
    return os.path.join(out_dir, f"{base_name}_{stem}.json")


# ==========================
# ENTRYPOINT FOR RunnerV2
# ==========================
def run():
    from compas.data import json_load
    from compas_rv.datastructures import Pattern

    """
    RunnerV2 calls run() with no args. :contentReference[oaicite:6]{index=6}
    Returns a small dict summary for console print.
    """
    try:
        if not os.path.isfile(SESSION_JSON):
            raise FileNotFoundError(f"SESSION_JSON not found: {SESSION_JSON}")

        session = load_raw_json(SESSION_JSON)

        # dtype exact names in your file include:
        # - compas_rv.datastructures/Pattern
        # - compas_rv.datastructures/FormDiagram
        # - compas_rv.datastructures/ThrustDiagram :contentReference[oaicite:7]{index=7}
        PAT_DTYPE = "compas_rv.datastructures/Pattern"
        pat_node = find_first_dtype(session, PAT_DTYPE)
        if not pat_node:
            raise Exception(f"Pattern dtype not found: {PAT_DTYPE}")

        session = json_load(SESSION_JSON)

                # ---------- Scene utilities (NO INDENT) ----------
        def iter_tree(node):
            yield node
            for c in getattr(node, "children", []) or []:
                yield from iter_tree(c)

        def get_items(node):
            items_attr = getattr(node, "items", None)
            if callable(items_attr):
                return items_attr()
            return items_attr or []

        def find_first_instance(root, cls):
            if isinstance(root, cls):
                return root
            for node in iter_tree(root):
                for it in get_items(node):
                    if isinstance(it, cls):
                        return it
            return None
        # ---------- end utilities ----------


        def find_first_instance(root, cls):
            if isinstance(root, cls):
                return root

            for node in iter_tree(root):
                items_attr = getattr(node, "items", None)

                # items may be a list OR a method returning a list
                if callable(items_attr):
                    items = items_attr()
                else:
                    items = items_attr

                if items:
                    for it in items:
                        if isinstance(it, cls):
                            return it
            return None

        pattern = find_first_instance(session, Pattern)
        if not pattern:
            raise Exception("Pattern not found in session (json_load succeeded).")

        

##
        #decoder = DataDecoder()
        #pattern = decoder.objectify({"dtype": pat_node["dtype"], "data": pat_node["data"]})
##
        # Build a fresh, consistent FormDiagram (this is the key step)
        form = FormDiagram.from_pattern(pattern)

        # Optional cleanup (helps avoid downstream weirdness)
        deleted_non_edges = 0
        deleted_iso = 0
        if DELETE_NON_EDGES:
            deleted_non_edges = delete_non_edges(form)
        if DELETE_ISOLATED_VERTICES:
            deleted_iso = delete_isolated_vertices(form)

        # Build thrust + solve
        thrust = ThrustDiagram.from_formdiagram(form)

        # Pick zmax
        zmax = ZMAX
        if zmax is None:
            zmax = read_zmax_from_session(session) or 2.0

        horizontal_nodal(thrust, kmax=H_KMAX, alpha=H_ALPHA)
        vertical_from_zmax(thrust, zmax=zmax, kmax=V_KMAX)

        intrados = extrados = None
        if MAKE_INTRA_EXTRA:
            half = 0.5 * float(THICKNESS)
            intrados = offset_mesh_along_normals(thrust, -half)
            extrados = offset_mesh_along_normals(thrust, +half)

        # Output files
        base = os.path.splitext(os.path.basename(SESSION_JSON))[0]
        out_dir = OUT_DIR or os.path.dirname(SESSION_JSON)

        p_form = out_path(base, out_dir, "FORM_REBUILT")
        p_thrust = out_path(base, out_dir, "THRUST_SOLVED")
        json_dump(form, p_form)
        json_dump(thrust, p_thrust)

        p_intra = p_extra = None
        if intrados and extrados:
            p_intra = out_path(base, out_dir, "INTRADOS")
            p_extra = out_path(base, out_dir, "EXTRADOS")
            json_dump(intrados, p_intra)
            json_dump(extrados, p_extra)

        print("DataDecoder methods:", [m for m in dir(DataDecoder()) if "dec" in m.lower() or "obj" in m.lower()])


        return {
            "ok": True,
            "session": SESSION_JSON,
            "out_dir": out_dir,
            "deleted_non_edges": deleted_non_edges,
            "deleted_isolated_vertices": deleted_iso,
            "zmax": zmax,
            "form_VEF": (form.number_of_vertices(), form.number_of_edges(), form.number_of_faces()),
            "thrust_VEF": (thrust.number_of_vertices(), thrust.number_of_edges(), thrust.number_of_faces()),
            "files": {
                "form": p_form,
                "thrust": p_thrust,
                "intrados": p_intra,
                "extrados": p_extra
            }
        }


    except Exception:
        return {
            "ok": False,
            "error": traceback.format_exc(),
            "session": SESSION_JSON
        }
