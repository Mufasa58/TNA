import os
import json
import glob
import traceback
import types

from compas.data import json_dump
from compas.geometry import add_vectors, scale_vector, normalize_vector

from compas_rv.datastructures import FormDiagram, ThrustDiagram
from compas_tna.diagrams import ForceDiagram
from compas_tna.equilibrium import horizontal_nodal, vertical_from_zmax


# ==========================
# CONFIG (edit these)
# ==========================
SESSION_JSON = r'/Users/mmg/dev/tna_playground/JSON Files/Pattern_simple_v2.json'
  # file OR folder
OUT_DIR = ""  # "" => same folder as input

# Solver controls
H_KMAX = 100
H_ALPHA = 100.0
V_KMAX = 300

# If None, try to read from RV session settings, else fallback 2.0
ZMAX = 2.4

# Intrados / extrados output
MAKE_INTRA_EXTRA = True
THICKNESS = 0.24

# Cleanups
DELETE_NON_EDGES = True
DELETE_ISOLATED_VERTICES = True

# Supports
SUPPORT_MODE = "auto"  # "auto" or "manual"
MANUAL_SUPPORT_VERTICES = []  # list of THRUST vertex ids (ints) when SUPPORT_MODE="manual"

# Optional: choose a corner strip instead of 4 points
AUTO_SUPPORT_USE_RING = False
AUTO_SUPPORT_RING_K = 1


# ==========================
# Helpers (RV JSON crawling)
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


# ==========================
# Helpers (graph / geometry)
# ==========================



def delete_non_edges(diagram):
    """Delete edges where RV stored _is_edge=False (these break topology assumptions)."""
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


def delete_non_vertices(diagram):
    """
    RV sometimes stores vertices with '_is_vertex': False.
    These can still be referenced by edges and will crash horizontal_nodal (KeyError in _k_i).
    """
    to_delete = []
    for v in list(diagram.vertices()):
        if diagram.vertex_attribute(v, "_is_vertex") is False:
            to_delete.append(v)

    for v in to_delete:
        if diagram.has_vertex(v):
            diagram.delete_vertex(v)

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


def k_ring_vertices(form, seeds, k=1):
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


# ==========================
# Edge indexing fix for KeyError
# ==========================

def reindex_edges_consecutively(diagram):
    edges = list(diagram.edges())
    for i, e in enumerate(edges):
        diagram.edge_attribute(e, "index", i)
    return {"m": len(edges), "min": 0 if edges else None, "max": len(edges) - 1 if edges else None}


def build_face_to_dual_vertex_map(form, force):
    """Try to map form face keys -> force vertex keys."""
    form_faces = set(form.faces())

    # Case 1: identity (common)
    if all((dv in form_faces) for dv in force.vertices()):
        return {f: f for f in form_faces if force.has_vertex(f)}

    # Case 2: look for a face id stored on force vertices
    candidate_attrs = ["face", "fkey", "_face", "primal_face", "source"]
    best = {}
    best_count = 0
    for attr in candidate_attrs:
        m = {}
        c = 0
        for dv in force.vertices():
            val = force.vertex_attribute(dv, attr)
            if val is None:
                continue
            try:
                fk = int(val)
            except Exception:
                continue
            if fk in form_faces:
                m[fk] = dv
                c += 1
        if c > best_count:
            best = m
            best_count = c
    return best


def sync_form_edge_indices_from_dual(form, force):
    """Assign form edge indices to match force edge indices based on dual adjacency."""
    # ensure force indices 0..m-1
    reindex_edges_consecutively(force)

    face_to_dv = build_face_to_dual_vertex_map(form, force)

    # dual edge -> index
    dual_index = {}
    for a, b in force.edges():
        idx = force.edge_attribute((a, b), "index")
        dual_index[frozenset((a, b))] = int(idx)

    # reset form indices
    for e in form.edges():
        form.edge_attribute(e, "index", None)

    mapped = 0
    for u, v in form.edges():
        faces = form.edge_faces((u, v))
        real = [f for f in faces if f is not None]
        if len(real) != 2:
            continue
        f1, f2 = real
        dv1 = face_to_dv.get(f1)
        dv2 = face_to_dv.get(f2)
        if dv1 is None or dv2 is None:
            continue
        idx = dual_index.get(frozenset((dv1, dv2)))
        if idx is None:
            continue

        # set BOTH orientations (some COMPAS versions store halfedge attrs directionally)
        try:
            form.edge_attribute((u, v), "index", idx)
        except Exception:
            pass
        try:
            form.edge_attribute((v, u), "index", idx)
        except Exception:
            pass

        mapped += 1

    # emulate ForceDiagram.ordered_edges(form) dict-build and check missing
    index_uv = {}
    for e in form.edges():
        idx = form.edge_attribute(e, "index")
        if idx is None:
            continue
        index_uv[int(idx)] = e

    m = force.number_of_edges()
    missing = [i for i in range(m) if i not in index_uv]

    return {
        "force_edges": m,
        "face_to_dv_size": len(face_to_dv),
        "mapped_form_edges": mapped,
        "index_uv_size": len(index_uv),
        "missing_count": len(missing),
        "missing_sample": missing[:10],
    }


def patch_force_ordered_edges(force):
    """Hard override to avoid KeyError if COMPAS internals still act weird."""

    def _ordered_edges(self, form):
        index_uv = {}
        for e in form.edges():
            idx = form.edge_attribute(e, "index")
            if idx is None:
                continue
            index_uv[int(idx)] = e
        return [index_uv[i] for i in range(self.number_of_edges())]

    force.ordered_edges = types.MethodType(_ordered_edges, force)


def _poly_area_xy(pts):
    # pts: list of (x,y)
    if len(pts) < 3:
        return 0.0
    a = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        a += x1 * y2 - x2 * y1
    return 0.5 * a

def remove_outer_face_if_present(mesh_like):
    """
    Heuristic: remove the single largest-area face that is mostly boundary vertices.
    This targets the unbounded/outside face that would otherwise get triangulated and 'cap' the vault.
    """
    bset = set(boundary_vertices(mesh_like))
    if not bset:
        return {"removed": False, "reason": "no boundary vertices"}

    best_f = None
    best_score = -1.0

    for f in list(mesh_like.faces()):
        vs = mesh_like.face_vertices(f)
        if not vs or len(vs) < 3:
            continue

        # must be strongly "boundary-ish"
        bcnt = sum(1 for v in vs if v in bset)
        frac = float(bcnt) / float(len(vs))

        # compute XY area magnitude
        pts = [(mesh_like.vertex_attribute(v, "x"), mesh_like.vertex_attribute(v, "y")) for v in vs]
        area = abs(_poly_area_xy(pts))

        # score: prioritize (big area) * (boundary fraction)
        score = area * frac

        if score > best_score:
            best_score = score
            best_f = (f, area, frac, len(vs))

    if best_f is None:
        return {"removed": False, "reason": "no candidate faces"}

    f, area, frac, nvs = best_f

    # guardrails: only delete if it *really* looks like the outside face
    # (large and mostly boundary vertices)
    if frac < 0.60:
        return {"removed": False, "reason": "best face not boundary-ish enough", "best": {"face": f, "area": area, "frac": frac, "n": nvs}}

    try:
        mesh_like.delete_face(f)
        return {"removed": True, "face": f, "area": area, "frac": frac, "n": nvs}
    except Exception as e:
        return {"removed": False, "reason": "delete_face failed", "error": str(e), "best": {"face": f, "area": area, "frac": frac, "n": nvs}}


# ==========================
# ENTRYPOINT FOR RunnerV2
# ==========================

def run():
    dbg = {}

    try:
        # --- resolve input file ---
        path = SESSION_JSON
        if os.path.isdir(path):
            candidates = glob.glob(os.path.join(path, "*.json"))
            if not candidates:
                raise FileNotFoundError(f"No .json files in folder: {path}")
            path = max(candidates, key=os.path.getmtime)
        if not os.path.isfile(path):
            raise FileNotFoundError(f"SESSION_JSON not found: {path}")

        dbg["session_path"] = path

        # --- load RV session as raw dict ---
        session_dict = load_raw_json(path)

        # --- find Pattern node ---
        PAT_DTYPE = "compas_rv.datastructures/Pattern"
        pat_node = find_first_dtype(session_dict, PAT_DTYPE)
        if not pat_node:
            raise Exception(f"Pattern dtype not found: {PAT_DTYPE}")

        # --- decode Pattern (DataDecoder route) ---
        from compas.data import DataDecoder

        payload = {"dtype": pat_node["dtype"], "data": pat_node["data"]}
        pattern = json.loads(json.dumps(payload), cls=DataDecoder)
        dbg["pattern_decode_method"] = "json.loads(..., cls=DataDecoder)"

        if pattern is None:
            raise Exception("Could not decode Pattern (DataDecoder returned None).")

        # --- build form from pattern ---
        form = FormDiagram.from_pattern(pattern)
        dbg["form_VEF_before_clean"] = (form.number_of_vertices(), form.number_of_edges(), form.number_of_faces())

        # --- supports ---
        if SUPPORT_MODE == "manual" and MANUAL_SUPPORT_VERTICES:
            supports = [int(v) for v in MANUAL_SUPPORT_VERTICES]
            apply_supports(form, supports)
            dbg["supports_mode"] = "manual"
            dbg["supports"] = supports
        else:
            bverts = boundary_vertices(form)
            supports4 = pick_4_corner_supports_by_quadrant(form, bverts)
            if AUTO_SUPPORT_USE_RING:
                supports = k_ring_vertices(form, supports4, k=AUTO_SUPPORT_RING_K)
            else:
                supports = supports4
            apply_supports(form, supports)
            dbg["supports_mode"] = "auto"
            dbg["supports"] = supports

        dbg["fixed_count"] = len([v for v in form.vertices() if form.vertex_attribute(v, "is_fixed")])

        # --- cleanup ---
        if DELETE_NON_EDGES:
            dbg["deleted_non_edges"] = delete_non_edges(form)
        dbg["deleted_non_vertices"] = delete_non_vertices(form)
        if DELETE_ISOLATED_VERTICES:
            dbg["deleted_isolated_vertices"] = delete_isolated_vertices(form)

        dbg["form_VEF_after_clean"] = (form.number_of_vertices(), form.number_of_edges(), form.number_of_faces())
        dbg["outer_face_removed"] = remove_outer_face_if_present(form)
        
        # --- build thrust ---
        try:
            thrust = form.copy(cls=ThrustDiagram)
            dbg["thrust_build"] = "form.copy(cls=ThrustDiagram)"
        except Exception:
            thrust = form.copy()
            dbg["thrust_build"] = "form.copy()"

        # --- defaults required by solvers ---
        q_defaulted = 0
        for e in thrust.edges():
            if thrust.edge_attribute(e, "q") is None:
                thrust.edge_attribute(e, "q", 1.0)
                q_defaulted += 1
        dbg["q_defaulted_edges"] = q_defaulted

        loads_defaulted = 0
        for v in thrust.vertices():
            if thrust.vertex_attribute(v, "px") is None:
                thrust.vertex_attribute(v, "px", 0.0)
                loads_defaulted += 1
            if thrust.vertex_attribute(v, "py") is None:
                thrust.vertex_attribute(v, "py", 0.0)
            if thrust.vertex_attribute(v, "pz") is None:
                thrust.vertex_attribute(v, "pz", 0.0)
        dbg["load_defaulted_vertices"] = loads_defaulted

        zmax = ZMAX
        if zmax is None:
            zmax = read_zmax_from_session(session_dict) or 2.0
        dbg["zmax_used"] = zmax

        # --- build force + sync indices to avoid KeyError ---
        force = ForceDiagram.from_formdiagram(thrust)
        dbg["force_VEF"] = (force.number_of_vertices(), force.number_of_edges(), force.number_of_faces())

        dbg["form_index_from_dual"] = sync_form_edge_indices_from_dual(thrust, force)

        # Only override ordered_edges if indices are still missing
        if dbg["form_index_from_dual"].get("missing_count", 0) > 0:
            patch_force_ordered_edges(force)

        # --- horizontal + vertical ---
        horizontal_nodal(thrust, force, kmax=H_KMAX, alpha=H_ALPHA)
        dbg["horizontal_ok"] = True

        vertical_from_zmax(thrust, zmax=zmax, kmax=V_KMAX)
        dbg["vertical_ok"] = True

        # --- forces ---
        # --- edge forces (N) + support reactions ---
        edge_forces = {}
        for u, v in thrust.edges():
            q = thrust.edge_attribute((u, v), "q") or 0.0
            xi = thrust.vertex_coordinates(u)
            xj = thrust.vertex_coordinates(v)
            dx = (xj[0] - xi[0], xj[1] - xi[1], xj[2] - xi[2])
            L = (dx[0]**2 + dx[1]**2 + dx[2]**2) ** 0.5
            N = q * L  # axial force magnitude
            thrust.edge_attribute((u, v), "N", float(N))
            edge_forces[f"{u}-{v}"] = {"q": float(q), "L": float(L), "N": float(N)}

        # reactions only on supports
        reactions = {}
        for i in thrust.vertices():
            if not thrust.vertex_attribute(i, "is_fixed"):
                continue

            xi = thrust.vertex_coordinates(i)

            # sum of q*(xj - xi) over neighbors
            sx = sy = sz = 0.0
            for j in thrust.vertex_neighbors(i):
                qij = thrust.edge_attribute((i, j), "q")
                if qij is None:
                    qij = thrust.edge_attribute((j, i), "q")
                qij = qij or 0.0

                xj = thrust.vertex_coordinates(j)
                sx += qij * (xj[0] - xi[0])
                sy += qij * (xj[1] - xi[1])
                sz += qij * (xj[2] - xi[2])

            px = thrust.vertex_attribute(i, "px") or 0.0
            py = thrust.vertex_attribute(i, "py") or 0.0
            pz = thrust.vertex_attribute(i, "pz") or 0.0

            Rx = -(sx + px)
            Ry = -(sy + py)
            Rz = -(sz + pz)

            thrust.vertex_attributes(i, ["Rx", "Ry", "Rz"], [float(Rx), float(Ry), float(Rz)])
            reactions[str(i)] = {"Rx": float(Rx), "Ry": float(Ry), "Rz": float(Rz)}

        dbg["edge_forces_computed"] = len(edge_forces)
        dbg["support_reactions_computed"] = len(reactions)


        # --- intrados/extrados ---
        intrados = extrados = None
        if MAKE_INTRA_EXTRA:
            half = 0.5 * float(THICKNESS)
            intrados = offset_mesh_along_normals(thrust, -half)
            extrados = offset_mesh_along_normals(thrust, +half)

        # --- save outputs ---
        base = os.path.splitext(os.path.basename(path))[0]
        out_dir = OUT_DIR or os.path.dirname(path)
        os.makedirs(out_dir, exist_ok=True)

        #here
        out_form = os.path.join(out_dir, f"{base}_FORM_REBUILT.json")
        out_thrust = os.path.join(out_dir, f"{base}_THRUST_SOLVED.json")
        out_force = os.path.join(out_dir, f"{base}_FORCE.json")
        json_dump(force, out_force)
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
                "force": out_force,
            },
            "dbg": dbg,
        }

    except Exception:
        return {
            "ok": False,
            "session": dbg.get("session_path", SESSION_JSON),
            "error": traceback.format_exc(),
            "dbg": dbg,
        }
