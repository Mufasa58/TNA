import os
import json
import glob
import traceback

from compas.data import json_dump, DataDecoder
from compas_rv.datastructures import FormDiagram, ThrustDiagram
from compas_tna.diagrams import ForceDiagram
from compas_tna.equilibrium import horizontal_nodal, vertical_from_zmax
from compas.geometry import add_vectors, scale_vector, normalize_vector


# =============================================================================
# CONFIG
# =============================================================================
# Can be a file OR a folder (newest *.json will be picked)
SESSION_JSON = r"/Users/mmg/dev/tna_playground/JSON Files/RhinoVAULT_Fan_Vault_V2_3.json"

OUT_DIR = ""  # "" => same folder as input json

# Support mode:
# - "auto": pick 4 outer corners from boundary (quadrant-farthest)
# - "manual": you select 4 Rhino point objects when script runs
SUPPORT_MODE = "manual"
MANUAL_BAKE_VERT_POINTS = True  # only used in SUPPORT_MODE="manual"
MANUAL_POINT_LAYER = "TNA_FORM_VERTS"

# Solver params
H_KMAX = 100
H_ALPHA = 100.0
V_KMAX = 300
ZMAX = None  # None => read from session if possible, else fallback 2.0

# Geometry outputs
MAKE_INTRA_EXTRA = True
THICKNESS = 0.12

# Topology cleanup
DELETE_NON_EDGES = False  # keep False until horizontal works reliably
DELETE_ISOLATED_VERTICES = True


# =============================================================================
# JSON helpers
# =============================================================================
def resolve_json_path(path_or_folder: str) -> str:
    if os.path.isdir(path_or_folder):
        candidates = glob.glob(os.path.join(path_or_folder, "*.json"))
        if not candidates:
            raise FileNotFoundError(f"No .json files in folder: {path_or_folder}")
        return max(candidates, key=os.path.getmtime)
    return path_or_folder


def load_raw_json(path: str):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def walk_nodes(node):
    if isinstance(node, dict):
        yield node
        for v in node.values():
            yield from walk_nodes(v)
    elif isinstance(node, list):
        for v in node:
            yield from walk_nodes(v)


def find_first_dtype(root, dtype_exact: str):
    for n in walk_nodes(root):
        if n.get("dtype") == dtype_exact and "data" in n:
            return n
    return None


def read_zmax_from_session(session_dict):
    for path in [
        ("scene", "data", "settings", "tna", "vertical_zmax"),
        ("settings", "tna", "vertical_zmax"),
        ("data", "settings", "tna", "vertical_zmax"),
    ]:
        try:
            d = session_dict
            for k in path:
                d = d[k]
            return float(d)
        except Exception:
            pass
    return None


def decode_compas_item(dtype: str, data):
    """Decode a COMPAS item from {dtype,data} using DataDecoder (works in your env via json.loads)."""
    payload = {"dtype": dtype, "data": data}
    return json.loads(json.dumps(payload), cls=DataDecoder)


# =============================================================================
# Diagram helpers
# =============================================================================
def delete_non_edges(diagram):
    to_delete = []
    for edge in list(diagram.edges()):
        if diagram.edge_attribute(edge, "_is_edge") is False:
            to_delete.append(edge)

    for edge in to_delete:
        try:
            diagram.delete_edge(edge)
        except Exception:
            try:
                u, v = edge
                diagram.delete_edge(u, v)
            except Exception:
                pass

    return len(to_delete)


def delete_isolated_vertices(diagram):
    isolated = [v for v in diagram.vertices() if diagram.vertex_degree(v) == 0]
    for v in isolated:
        diagram.delete_vertex(v)
    return len(isolated)


def reindex_edges_consecutively(diagram):
    """Force edge 'index' to be 0..m-1 (prevents ForceDiagram.ordered_edges KeyError)."""
    edges = list(diagram.edges())
    for i, e in enumerate(edges):
        diagram.edge_attribute(e, "index", i)

    idx = [diagram.edge_attribute(e, "index") for e in edges]
    missing = set(range(len(edges))) - set(idx)
    return {
        "m": len(edges),
        "missing_count": len(missing),
        "min": min(idx) if idx else None,
        "max": max(idx) if idx else None,
    }


def reindex_interior_edges(form):
    """
    Index ONLY interior edges (those with 2 adjacent faces) as 0..m-1.
    Boundary edges get index=None.
    This matches ForceDiagram.ordered_edges(form) expectations.
    """
    interior = []
    boundary_like = 0

    for e in list(form.edges()):
        faces = form.edge_faces(e)
        real = [f for f in faces if f is not None]
        if len(real) == 2:
            interior.append(e)
        else:
            boundary_like += 1
            form.edge_attribute(e, "index", None)

    for i, e in enumerate(interior):
        form.edge_attribute(e, "index", i)

    return {
        "total_edges": form.number_of_edges(),
        "interior_edges": len(interior),
        "boundary_like_edges": boundary_like,
        "index_min": 0 if interior else None,
        "index_max": (len(interior) - 1) if interior else None,
    }

def reindex_form_edges_from_force(form, force):
    """
    ForceDiagram edges store which form edge they correspond to (usually under 'uv').
    Use that mapping to index ONLY the dual-able form edges consecutively.
    """
    uv_edges = []
    for e in force.edges():
        uv = force.edge_attribute(e, "uv")
        if uv is None:
            continue
        # uv can be (u,v) or [u,v]
        uv = tuple(uv)
        uv_edges.append(uv)

    # remove duplicates while keeping order
    seen = set()
    ordered_uv = []
    for uv in uv_edges:
        if uv not in seen and (uv[1], uv[0]) not in seen:
            ordered_uv.append(uv)
            seen.add(uv)

    # reset all indices
    for e in form.edges():
        form.edge_attribute(e, "index", None)

    # apply consecutive indices to the dual edges only
    for i, uv in enumerate(ordered_uv):
        u, v = uv
        if form.has_edge((u, v)):
            form.edge_attribute((u, v), "index", i)
        elif form.has_edge((v, u)):
            form.edge_attribute((v, u), "index", i)
        else:
            # mapping refers to an edge not present -> will break, record it
            pass

    return {"dual_edges": len(ordered_uv), "index_max": len(ordered_uv) - 1}

# =============================================================================
# Support selection
# =============================================================================
def boundary_vertices(form):
    """Boundary vertices (outer + inner)."""
    b = set()
    for e in form.edges():
        faces = form.edge_faces(e)
        real = [f for f in faces if f is not None]
        if len(real) == 1:
            u, v = e
            b.add(u)
            b.add(v)
    return list(b)


def xy(form, v):
    return (form.vertex_attribute(v, "x"), form.vertex_attribute(v, "y"))


def quadrant_key(cx, cy, x, y):
    if x >= cx and y >= cy:
        return "NE"
    if x < cx and y >= cy:
        return "NW"
    if x < cx and y < cy:
        return "SW"
    return "SE"


def pick_4_corner_supports_by_quadrant(form, candidates):
    xs = [xy(form, v)[0] for v in candidates]
    ys = [xy(form, v)[1] for v in candidates]
    cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)

    best = {"NE": (None, -1), "NW": (None, -1), "SW": (None, -1), "SE": (None, -1)}
    for v in candidates:
        x, y = xy(form, v)
        q = quadrant_key(cx, cy, x, y)
        d2 = (x - cx) ** 2 + (y - cy) ** 2
        if d2 > best[q][1]:
            best[q] = (v, d2)

    supports = [best["NW"][0], best["NE"][0], best["SE"][0], best["SW"][0]]
    return [v for v in supports if v is not None]


def apply_supports(form, supports):
    for v in form.vertices():
        form.vertex_attribute(v, "is_fixed", False)
    for v in supports:
        form.vertex_attribute(v, "is_fixed", True)


def bake_form_vertices_as_points(form, layer_name):
    import rhinoscriptsyntax as rs

    if not rs.IsLayer(layer_name):
        rs.AddLayer(layer_name)
    rs.CurrentLayer(layer_name)

    guids = []
    for v in form.vertices():
        x = form.vertex_attribute(v, "x")
        y = form.vertex_attribute(v, "y")
        z = form.vertex_attribute(v, "z") or 0.0
        g = rs.AddPoint(x, y, z)
        if g:
            guids.append(g)
    return guids


def set_supports_from_selected_points(form):
    import rhinoscriptsyntax as rs

    guids = rs.GetObjects("Select 4 support point objects (preselect ok) then Enter", rs.filter.point, preselect=True)
    if not guids:
        return []

    picked = [rs.PointCoordinates(g) for g in guids]
    verts = list(form.vertices())
    supports = []

    for px, py, pz in picked:
        best = None
        bestd = 1e99
        for v in verts:
            x = form.vertex_attribute(v, "x")
            y = form.vertex_attribute(v, "y")
            d = (x - px) ** 2 + (y - py) ** 2
            if d < bestd:
                bestd = d
                best = v
        if best is not None and best not in supports:
            supports.append(best)

    apply_supports(form, supports)
    return supports


# =============================================================================
# Intrados / extrados
# =============================================================================
def mesh_vertex_normals(mesh):
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


# =============================================================================
# RunnerV2 entry point
# =============================================================================
def run():
    dbg = {}
    try:
        path = resolve_json_path(SESSION_JSON)
        if not os.path.isfile(path):
            raise FileNotFoundError(f"SESSION_JSON not found: {path}")
        dbg["session_path"] = path

        session_dict = load_raw_json(path)

        PAT_DTYPE = "compas_rv.datastructures/Pattern"
        pat_node = find_first_dtype(session_dict, PAT_DTYPE)
        if not pat_node:
            raise Exception(f"Pattern dtype not found: {PAT_DTYPE}")

        pattern = decode_compas_item(pat_node["dtype"], pat_node["data"])
        dbg["pattern_decode_method"] = "json.loads(..., cls=DataDecoder)"

        import compas, compas_tna, compas_rv
        dbg["compas_file"] = compas.__file__
        dbg["compas_tna_file"] = compas_tna.__file__
        dbg["compas_rv_file"] = compas_rv.__file__
        dbg["force_class_module"] = ForceDiagram.__module__
        dbg["force_class_mro"] = [c.__module__ + "." + c.__name__ for c in ForceDiagram.__mro__]


        form = FormDiagram.from_pattern(pattern)
        dbg["form_VEF"] = (form.number_of_vertices(), form.number_of_edges(), form.number_of_faces())

        # Supports
        if SUPPORT_MODE.lower() == "manual":
            if MANUAL_BAKE_VERT_POINTS:
                dbg["baked_points"] = len(bake_form_vertices_as_points(form, MANUAL_POINT_LAYER))
            supports = set_supports_from_selected_points(form)
            dbg["supports_mode"] = "manual"
        else:
            bverts = boundary_vertices(form)
            supports = pick_4_corner_supports_by_quadrant(form, bverts)
            apply_supports(form, supports)
            dbg["supports_mode"] = "auto"

        dbg["supports"] = supports
        dbg["fixed_count"] = len([v for v in form.vertices() if form.vertex_attribute(v, "is_fixed")])

        # Cleanup
        if DELETE_NON_EDGES:
            dbg["deleted_non_edges"] = delete_non_edges(form)
        if DELETE_ISOLATED_VERTICES:
            dbg["deleted_isolated_vertices"] = delete_isolated_vertices(form)
        dbg["form_VEF_after_clean"] = (form.number_of_vertices(), form.number_of_edges(), form.number_of_faces())

        # Thrust diagram (copy)
        try:
            thrust = form.copy(cls=ThrustDiagram)
            dbg["thrust_build"] = "form.copy(cls=ThrustDiagram)"
        except Exception as e:
            thrust = form.copy()
            dbg["thrust_build"] = "form.copy()"
            dbg["thrust_copy_cls_error"] = str(e)

       # CRITICAL: index only interior edges of the form/thrust
        dbg["thrust_interior_index"] = reindex_interior_edges(thrust)

        # build force from thrust (do NOT reindex force afterwards)
        force = ForceDiagram.from_formdiagram(thrust)
        dbg["force_VEF"] = (force.number_of_vertices(), force.number_of_edges(), force.number_of_faces())

        horizontal_nodal(thrust, force, kmax=H_KMAX, alpha=H_ALPHA)
        dbg["horizontal_ok"] = True


        # Horizontal equilibrium
        horizontal_nodal(thrust, force, kmax=H_KMAX, alpha=H_ALPHA)
        dbg["horizontal_ok"] = True

        # Vertical equilibrium
        zmax = ZMAX if ZMAX is not None else (read_zmax_from_session(session_dict) or 2.0)
        dbg["zmax_used"] = zmax

        vertical_from_zmax(thrust, zmax=zmax, kmax=V_KMAX)
        dbg["vertical_ok"] = True

        dbg["thrust_VEF"] = (thrust.number_of_vertices(), thrust.number_of_edges(), thrust.number_of_faces())

        intrados = extrados = None
        if MAKE_INTRA_EXTRA:
            half = 0.5 * float(THICKNESS)
            intrados = offset_mesh_along_normals(thrust, -half)
            extrados = offset_mesh_along_normals(thrust, +half)

        # Save
        base = os.path.splitext(os.path.basename(path))[0]
        out_dir = OUT_DIR or os.path.dirname(path)
        os.makedirs(out_dir, exist_ok=True)

        out_form = os.path.join(out_dir, f"{base}_FORM_REBUILT.json")
        out_thrust = os.path.join(out_dir, f"{base}_THRUST_SOLVED.json")
        json_dump(form, out_form)
        json_dump(thrust, out_thrust)

        out_intra = out_extra = None
        if intrados and extrados:
            out_intra = os.path.join(out_dir, f"{base}_INTRADOS.json")
            out_extra = os.path.join(out_dir, f"{base}_EXTRADOS.json")
            json_dump(intrados, out_intra)
            json_dump(extrados, out_extra)

        return {
            "ok": True,
            "session": path,
            "out_dir": out_dir,
            "files": {"form": out_form, "thrust": out_thrust, "intrados": out_intra, "extrados": out_extra},
            "dbg": dbg,
        }

    except Exception:
        return {"ok": False, "session": dbg.get("session_path", SESSION_JSON), "error": traceback.format_exc(), "dbg": dbg}
