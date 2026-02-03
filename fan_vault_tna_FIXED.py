import os
import json
import glob
import traceback

from compas.data import json_dump
from compas_rv.datastructures import FormDiagram, ThrustDiagram
from compas_tna.equilibrium import horizontal_nodal, vertical_from_zmax

from compas.geometry import add_vectors, scale_vector, normalize_vector


# ==========================
# CONFIG
# ==========================
SESSION_JSON = r'/Users/mmg/dev/tna_playground/JSON Files/RhinoVAULT_Fan_Vault_V2_3.json' # folder OR file
OUT_DIR = ""  # "" = same folder as input

H_KMAX = 100
H_ALPHA = 100.0
V_KMAX = 300
ZMAX = None  # None => read from session if possible, else fallback 2.0

MAKE_INTRA_EXTRA = True
THICKNESS = 0.12

DELETE_NON_EDGES = True
DELETE_ISOLATED_VERTICES = True


# ==========================
# Helpers (raw dict)
# ==========================
def load_raw_json(path):
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

def find_first_dtype(root, dtype_exact):
    for n in walk_nodes(root):
        if n.get("dtype") == dtype_exact and "data" in n:
            return n
    return None

def read_zmax_from_session(session_dict):
    # robust: try a few common locations
    for path in [
        ("scene","data","settings","tna","vertical_zmax"),
        ("settings","tna","vertical_zmax"),
        ("data","settings","tna","vertical_zmax"),
    ]:
        try:
            d = session_dict
            for k in path:
                d = d[k]
            return float(d)
        except Exception:
            pass
    return None

def delete_non_edges(diagram):
    # Robust across COMPAS versions (has_edge signature differs)
    to_delete = []
    for edge in list(diagram.edges()):
        # edge is typically a tuple (u, v)
        if diagram.edge_attribute(edge, "_is_edge") is False:
            to_delete.append(edge)

    for edge in to_delete:
        # safest: just attempt delete; edge came from diagram.edges() anyway
        try:
            diagram.delete_edge(edge)
        except Exception:
            # older versions may want u, v separately
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

def boundary_vertices(form):
    """Return boundary vertices (both outer + inner boundaries)."""
    b = set()
    for e in form.edges():
        faces = form.edge_faces(e)
        real = [f for f in faces if f is not None]
        if len(real) == 1:
            u, v = e
            b.add(u); b.add(v)
    return list(b)

def xy(form, v):
    return (form.vertex_attribute(v, "x"), form.vertex_attribute(v, "y"))

def quadrant_key(cx, cy, x, y):
    # NE, NW, SW, SE
    if x >= cx and y >= cy: return "NE"
    if x <  cx and y >= cy: return "NW"
    if x <  cx and y <  cy: return "SW"
    return "SE"

def pick_4_corner_supports_by_quadrant(form, candidates):
    # centroid
    xs = [xy(form,v)[0] for v in candidates]
    ys = [xy(form,v)[1] for v in candidates]
    cx, cy = sum(xs)/len(xs), sum(ys)/len(ys)

    # pick farthest point in each quadrant
    best = {"NE": (None, -1), "NW": (None, -1), "SW": (None, -1), "SE": (None, -1)}
    for v in candidates:
        x, y = xy(form, v)
        q = quadrant_key(cx, cy, x, y)
        d2 = (x-cx)**2 + (y-cy)**2
        if d2 > best[q][1]:
            best[q] = (v, d2)

    supports = [best["NW"][0], best["NE"][0], best["SE"][0], best["SW"][0]]
    supports = [v for v in supports if v is not None]
    return supports

def k_ring_vertices(form, seeds, k=1):
    """Graph BFS expansion k steps from seed vertices."""
    current = set(seeds)
    visited = set(seeds)
    for _ in range(k):
        nxt = set()
        for v in current:
            for nbr in form.vertex_neighbors(v):
                if nbr not in visited:
                    nxt.add(nbr)
        visited |= nxt
        current = nxt
    return list(visited)

def apply_supports(form, supports):
    for v in form.vertices():
        form.vertex_attribute(v, "is_fixed", False)
    for v in supports:
        form.vertex_attribute(v, "is_fixed", True)
def reindex_edges_consecutively(diagram):
    """Set diagram edge 'index' = 0..m-1 for its own edges."""
    edges = list(diagram.edges())
    for i, e in enumerate(edges):
        diagram.edge_attribute(e, "index", i)
    return {"m": len(edges), "min": 0 if edges else None, "max": len(edges) - 1 if edges else None}


def build_face_to_dual_vertex_map(form, force):
    """
    Map form face keys -> force vertex keys.
    Tries a few common attribute names, otherwise falls back to identity when possible.
    """
    form_faces = set(form.faces())

    # 1) Identity mapping works when dual vertex keys are face keys
    ok = True
    for dv in force.vertices():
        if dv not in form_faces:
            ok = False
            break
    if ok:
        return {f: f for f in form_faces if force.has_vertex(f)}

    # 2) Try common attribute names on force vertices
    candidate_attrs = ["face", "fkey", "_face", "primal_face", "source", "key"]
    best = {}
    best_count = 0

    for attr in candidate_attrs:
        mapping = {}
        count = 0
        for dv in force.vertices():
            val = force.vertex_attribute(dv, attr)
            if val is None:
                continue
            try:
                fk = int(val)
            except Exception:
                continue
            if fk in form_faces:
                mapping[fk] = dv
                count += 1
        if count > best_count:
            best = mapping
            best_count = count

    return best  # may be partial


def reindex_form_edges_from_dual(form, force):
    """
    After force exists, assign form edge indices so that ForceDiagram.ordered_edges(form) works.
    Strategy:
      - Give each force edge a consecutive index 0..m-1
      - For each form edge with 2 adjacent faces f1,f2:
          find corresponding dual vertices dv1,dv2
          if force has edge(dv1,dv2): assign form edge index = force edge index
    """
    # 1) Ensure force edges have clean indices
    reindex_edges_consecutively(force)

    # 2) Build face->dualvertex mapping
    face_to_dv = build_face_to_dual_vertex_map(form, force)

    # 3) Dual edge -> index lookup
    dual_index = {}
    for a, b in force.edges():
        idx = force.edge_attribute((a, b), "index")
        dual_index[frozenset((a, b))] = idx

    # 4) Reset all form indices
    for e in form.edges():
        form.edge_attribute(e, "index", None)

    # 5) Assign indices where we can
    mapped = 0
    for u, v in form.edges():
        faces = form.edge_faces((u, v))
        real = [f for f in faces if f is not None]
        if len(real) != 2:
            continue

        f1, f2 = real[0], real[1]
        dv1 = face_to_dv.get(f1)
        dv2 = face_to_dv.get(f2)
        if dv1 is None or dv2 is None:
            continue

        idx = dual_index.get(frozenset((dv1, dv2)))
        if idx is None:
            continue

        form.edge_attribute((u, v), "index", idx)
        mapped += 1

    # 6) Check missing indices
    m = force.number_of_edges()
    present = set()
    for e in form.edges():
        idx = form.edge_attribute(e, "index")
        if isinstance(idx, int):
            present.add(idx)

    missing = sorted(set(range(m)) - present)

    return {
        "force_edges": m,
        "face_to_dv_size": len(face_to_dv),
        "mapped_form_edges": mapped,
        "missing_count": len(missing),
        "missing_sample": missing[:10],
    }



# ==========================
# Entry point for RunnerV2
# ==========================
def run():
    dbg = {}

    try:
        # 1) Resolve input (folder -> newest json)
        path = SESSION_JSON
        if os.path.isdir(path):
            candidates = glob.glob(os.path.join(path, "*.json"))
            if not candidates:
                raise FileNotFoundError(f"No .json files in folder: {path}")
            path = max(candidates, key=os.path.getmtime)

        if not os.path.isfile(path):
            raise FileNotFoundError(f"SESSION_JSON not found: {path}")

        dbg["session_path"] = path

        # 2) Load raw dict and find Pattern node
        session_dict = load_raw_json(path)

        PAT_DTYPE = "compas_rv.datastructures/Pattern"
        pat_node = find_first_dtype(session_dict, PAT_DTYPE)
        if not pat_node:
            dbg["available_dtypes_sample"] = sorted(
                {n.get("dtype") for n in walk_nodes(session_dict) if isinstance(n.get("dtype"), str)}
            )[:50]
            raise Exception(f"Pattern dtype not found: {PAT_DTYPE}")

        # 3) Decode the Pattern in a version-safe way
        pattern = None

        # 3) Decode the Pattern in a version-safe way
        from compas.data import DataDecoder

        payload = {"dtype": pat_node["dtype"], "data": pat_node["data"]}

        try:
            pattern = json.loads(json.dumps(payload), cls=DataDecoder)
            dbg["pattern_decode_method"] = "json.loads(..., cls=DataDecoder)"
        except Exception as e:
            dbg["pattern_decode_method"] = "FAILED"
            dbg["pattern_decode_error"] = str(e)
            pattern = None

        if pattern is None:
            raise Exception("Could not decode Pattern (DataDecoder failed).")


        # 3b) Fallback: Pattern.from_data (if available)
        if pattern is None:
            from compas_rv.datastructures import Pattern
            dbg["Pattern_has_from_data"] = hasattr(Pattern, "from_data")
            if hasattr(Pattern, "from_data"):
                pattern = Pattern.from_data(pat_node["data"])

        if pattern is None:
            raise Exception("Could not decode Pattern (no decoder available in this environment).")

        # 4) Build clean FormDiagram from Pattern
        form = FormDiagram.from_pattern(pattern)

        fixed_from_pattern = 0
        for v in form.vertices():
            if pattern.has_vertex(v):
                is_fixed = bool(pattern.vertex_attribute(v, "is_fixed") or pattern.vertex_attribute(v, "is_support"))
                if is_fixed:
                    form.vertex_attribute(v, "is_fixed", True)
                    fixed_from_pattern += 1

        dbg["fixed_from_pattern"] = fixed_from_pattern

        dbg["form_VEF_before_clean"] = (form.number_of_vertices(), form.number_of_edges(), form.number_of_faces())

        # 4.5) Auto-apply supports if none exist
        bverts = boundary_vertices(form)

        # If you have an inner hole, bverts includes inner + outer boundary.
        # This quadrant method usually still selects the OUTER corners because they are farthest from centroid.
        supports = pick_4_corner_supports_by_quadrant(form, bverts)

        # If you want 1-ring (first “row” around each corner):
        supports_strip = k_ring_vertices(form, supports, k=1)

        apply_supports(form, supports)          # 4 points only (recommended)
        # apply_supports(form, supports_strip)  # corner strip (use only if needed)

        dbg["supports_4"] = supports
        dbg["supports_strip_count"] = len(supports_strip)


        # 5) Cleanup
        deleted_non = 0
        deleted_iso = 0
        if DELETE_NON_EDGES:
            deleted_non = delete_non_edges(form)
        if DELETE_ISOLATED_VERTICES:
            deleted_iso = delete_isolated_vertices(form)

        dbg["deleted_non_edges"] = deleted_non
        dbg["deleted_isolated_vertices"] = deleted_iso
        dbg["form_VEF_after_clean"] = (form.number_of_vertices(), form.number_of_edges(), form.number_of_faces())

        # 6) Build thrust diagram (version-safe)
        thrust = None

        # Try: form.copy(cls=ThrustDiagram) if your FormDiagram supports it
        try:
            thrust = form.copy(cls=ThrustDiagram)
            dbg["thrust_build"] = "form.copy(cls=ThrustDiagram)"
        except Exception as e:
            dbg["thrust_copy_cls_error"] = str(e)

        # Fallback: plain copy (still works as datastructure for equilibrium solvers)
        if thrust is None:
            thrust = form.copy()
            dbg["thrust_build"] = "form.copy()"

                # Ensure supports exist (RV sessions usually have is_fixed / is_support on vertices)
        # If none exist, the solver will fail.
        fixed = [v for v in thrust.vertices() if thrust.vertex_attribute(v, "is_fixed")]
        dbg["fixed_count"] = len(fixed)

        from compas_tna.diagrams import ForceDiagram

        force = ForceDiagram.from_formdiagram(thrust)
        dbg["force_VEF"] = (force.number_of_vertices(), force.number_of_edges(), force.number_of_faces())

        dbg["form_index_from_dual"] = reindex_form_edges_from_dual(thrust, force)

        horizontal_nodal(thrust, force, kmax=H_KMAX, alpha=H_ALPHA)
        dbg["horizontal_ok"] = True

        # Ensure force densities q exist
        # Some sessions store 'q' already, but we set a default if missing.
        q_missing = 0
        for e in thrust.edges():
            if thrust.edge_attribute(e, "q") is None:
                thrust.edge_attribute(e, "q", 1.0)
                q_missing += 1
        dbg["q_defaulted_edges"] = q_missing

        # Ensure nodal loads exist (px,py,pz); default to 0 if missing
        load_missing = 0
        for v in thrust.vertices():
            if thrust.vertex_attribute(v, "px") is None:
                thrust.vertex_attribute(v, "px", 0.0); load_missing += 1
            if thrust.vertex_attribute(v, "py") is None:
                thrust.vertex_attribute(v, "py", 0.0)
            if thrust.vertex_attribute(v, "pz") is None:
                thrust.vertex_attribute(v, "pz", 0.0)
        dbg["load_defaulted_vertices"] = load_missing


        zmax = ZMAX
        if zmax is None:
            zmax = read_zmax_from_session(session_dict) or 2.0
        dbg["zmax_used"] = zmax

        from compas_tna.diagrams import ForceDiagram
        force = ForceDiagram.from_formdiagram(thrust)  # or from_formdiagram(form) — both share same topo
        dbg["force_VEF"] = (force.number_of_vertices(), force.number_of_edges(), force.number_of_faces())


        horizontal_nodal(thrust, force, kmax=H_KMAX, alpha=H_ALPHA)
        
        vertical_from_zmax(thrust, zmax=zmax, kmax=V_KMAX)

        dbg["thrust_VEF"] = (thrust.number_of_vertices(), thrust.number_of_edges(), thrust.number_of_faces())

        # 7) Intrados/extrados
        intrados = extrados = None
        if MAKE_INTRA_EXTRA:
            half = 0.5 * float(THICKNESS)
            intrados = offset_mesh_along_normals(thrust, -half)
            extrados = offset_mesh_along_normals(thrust, +half)

        # 8) Save outputs
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
            "files": {
                "form": out_form,
                "thrust": out_thrust,
                "intrados": out_intra,
                "extrados": out_extra,
            },
            "dbg": dbg
        }

    except Exception:
        return {
            "ok": False,
            "session": dbg.get("session_path", SESSION_JSON),
            "error": traceback.format_exc(),
            "dbg": dbg
        }
def sync_form_edge_indices_from_force(form, force):
    """Assign consecutive edge indices on the FORM/THRUST diagram for the dual-able edges.

    compas_tna's ForceDiagram.ordered_edges(form) expects that the form edges that participate
    in the dual (i.e. the edges referenced by force edge attribute 'uv') have edge attribute
    'index' exactly 0..(m-1) where m = force.number_of_edges().

    We therefore:
    - reset all form edge indices to None
    - iterate force edges, read their 'uv' mapping, and set form.edge_attribute(uv, 'index', i)
      with i consecutive.
    """
    # reset indices
    for e in form.edges():
        form.edge_attribute(e, "index", None)

    missing = 0
    mapped = 0
    for i, fe in enumerate(list(force.edges())):
        uv = force.edge_attribute(fe, "uv")
        if uv is None:
            missing += 1
            continue
        try:
            u, v = uv
        except Exception:
            uv = tuple(uv)
            u, v = uv[0], uv[1]

        # form.has_edge signature differs across versions; try both
        ok = False
        try:
            if form.has_edge((u, v)):
                form.edge_attribute((u, v), "index", i)
                ok = True
        except Exception:
            pass
        if not ok:
            try:
                if form.has_edge((v, u)):
                    form.edge_attribute((v, u), "index", i)
                    ok = True
            except Exception:
                pass

        if ok:
            mapped += 1
        else:
            missing += 1

    return {"force_edges": force.number_of_edges(), "mapped": mapped, "missing": missing, "index_max": mapped - 1 if mapped else None}



