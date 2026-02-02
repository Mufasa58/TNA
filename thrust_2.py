"""
thrust_1.py  —  Funicular Floor (3x3 bays) + columns + 9-piece segmentation per bay
Rhino 8 / Python 3.9 / COMPAS 2.x / compas_tna 0.7.x

Outputs (in Rhino layers):
- TNA_FORM_MESH    : ONE Rhino mesh of the lifted thrust network (fast, clean)
- TNA_SEAMS        : seam curves draped onto the thrust surface (9 pieces per bay)
- TNA_COLUMNS      : simple column lines at supports
- TNA_FORCE        : decimated force diagram (2D), offset in +X
- TNA_FORM_FORCE   : (optional) decimated colored form edges by |force| proxy

Notes:
- This is NOT RhinoVault UI. Pure script.
- “Segmentation” is a geometric overlay here (joints do not yet modify equilibrium topology).
- For “engineering meaning”, we compute seam-traffic proxy: sum(|edge force|) crossing seams per bay.

If you see “only a grid”: you are likely drawing edges instead of the mesh.
This script draws ONE mesh for the form.
"""

from __future__ import annotations

import math
from typing import Dict, Tuple, List

import Rhino
import scriptcontext as sc
import System.Drawing as SD

from compas.datastructures import Mesh
from compas_tna.diagrams import FormDiagram, ForceDiagram
from compas_tna.equilibrium import horizontal_nodal, vertical_from_zmax


# ==============================================================================
# PARAMETERS
# ==============================================================================

PARAMS: Dict[str, float] = {
    # Geometry (meters, consistent with Rhino doc in meters)
    "bay": 7.0,
    "bays_x": 3,
    "bays_y": 3,
    "subdiv_per_bay": 20,   # start here; 70 is heavy (you can increase later)
    "zmax": 2.0,

    # Loads (abstract nodal load)
    "w": 10.0,

    # Solver
    "alpha": 100.0,
    "kmax_hor": 200,

    # Visualization toggles
    "clear_layers_each_run": True,
    "draw_form_mesh": True,
    "draw_form_edges_colored": False,     # optional debug
    "draw_every_nth_colored_edge": 20,

    "draw_force_diagram": True,
    "draw_every_nth_force_edge": 40,
    "force_offset_x": 30.0,               # offset force diagram in +X (m)
    "force_offset_y": 0.0,
    "force_scale": 1.0,

    "draw_columns": True,
    "column_height": 3.0,
    "column_draw_as_lines": True,

    # Segmentation (9 pieces per bay)
    "draw_seams": True,
    "oblique_offset_deg": 45.0,           # seam rotation away from local force direction
    "r_frac": 0.22,                       # inner octagon radius fraction of bay
    "seam_sample_step": 2,                # drape seams using every N seam vertices

    # Layers
    "layer_form_mesh": "TNA_FORM_MESH",
    "layer_force": "TNA_FORCE",
    "layer_seams": "TNA_SEAMS",
    "layer_columns": "TNA_COLUMNS",
    "layer_form_force": "TNA_FORM_FORCE",
}


# ==============================================================================
# RHINO HELPERS
# ==============================================================================

def _ensure_layer(name: str) -> int:
    doc = sc.doc
    layer = doc.Layers.FindName(name)
    if layer is None:
        layer = Rhino.DocObjects.Layer()
        layer.Name = name
        return doc.Layers.Add(layer)
    return layer.Index


def clear_layer_objects(layer_name: str) -> None:
    doc = sc.doc
    layer = doc.Layers.FindName(layer_name)
    if layer is None:
        return
    idx = layer.Index
    ids = [obj.Id for obj in doc.Objects if obj.Attributes.LayerIndex == idx]
    for gid in ids:
        doc.Objects.Delete(gid, True)


def _add_line(a, b, layer: str, color: SD.Color | None = None, width: int | None = None):
    doc = sc.doc
    idx = _ensure_layer(layer)

    p0 = Rhino.Geometry.Point3d(*a)
    p1 = Rhino.Geometry.Point3d(*b)
    line = Rhino.Geometry.Line(p0, p1)

    attr = Rhino.DocObjects.ObjectAttributes()
    attr.LayerIndex = idx
    if color is not None:
        attr.ObjectColor = color
        attr.ColorSource = Rhino.DocObjects.ObjectColorSource.ColorFromObject
    if width is not None:
        attr.PlotWeight = width

    doc.Objects.AddLine(line, attr)


def _add_point(p, layer: str, color: SD.Color | None = None):
    doc = sc.doc
    idx = _ensure_layer(layer)
    pt = Rhino.Geometry.Point3d(*p)

    attr = Rhino.DocObjects.ObjectAttributes()
    attr.LayerIndex = idx
    if color is not None:
        attr.ObjectColor = color
        attr.ColorSource = Rhino.DocObjects.ObjectColorSource.ColorFromObject

    doc.Objects.AddPoint(pt, attr)


def _add_textdot(text: str, p, layer: str):
    doc = sc.doc
    idx = _ensure_layer(layer)
    dot = Rhino.Geometry.TextDot(text, Rhino.Geometry.Point3d(*p))
    attr = Rhino.DocObjects.ObjectAttributes()
    attr.LayerIndex = idx
    doc.Objects.AddTextDot(dot, attr)


def _add_mesh(rm: Rhino.Geometry.Mesh, layer: str, color: SD.Color | None = None):
    doc = sc.doc
    idx = _ensure_layer(layer)
    attr = Rhino.DocObjects.ObjectAttributes()
    attr.LayerIndex = idx
    if color is not None:
        attr.ObjectColor = color
        attr.ColorSource = Rhino.DocObjects.ObjectColorSource.ColorFromObject
    doc.Objects.AddMesh(rm, attr)


def _colormap_blue_red(t: float) -> SD.Color:
    t = max(0.0, min(1.0, t))
    r = int(round(255 * t))
    b = int(round(255 * (1.0 - t)))
    return SD.Color.FromArgb(r, 0, b)


# ==============================================================================
# PHASE 1 — TOPOLOGY (meshgrid -> FormDiagram)
# ==============================================================================

def make_form(params: Dict[str, float]) -> FormDiagram:
    bay = float(params["bay"])
    bx = int(params["bays_x"])
    by = int(params["bays_y"])
    subdiv = int(params["subdiv_per_bay"])

    nx = bx * subdiv
    ny = by * subdiv
    dx = bay / subdiv

    mesh = Mesh.from_meshgrid(dx=dx, nx=nx, dy=dx, ny=ny)
    form = FormDiagram.from_mesh(mesh)

    # enforce z=0 explicitly
    for v in form.vertices():
        x, y, _ = form.vertex_coordinates(v)
        form.vertex_attributes(v, ["x", "y", "z"], [x, y, 0.0])

    return form


# ==============================================================================
# PHASE 2 — SUPPORTS (columns at grid intersections)
# ==============================================================================

def apply_supports(form: FormDiagram, params: Dict[str, float]) -> List[int]:
    bay = float(params["bay"])
    bx = int(params["bays_x"])
    by = int(params["bays_y"])
    tol = 1e-9

    col_coords = set()
    for i in range(bx + 1):
        for j in range(by + 1):
            col_coords.add((i * bay, j * bay))

    supports = []
    for v in form.vertices():
        x, y, _ = form.vertex_coordinates(v)
        for (cx, cy) in col_coords:
            if abs(x - cx) < tol and abs(y - cy) < tol:
                form.vertex_attribute(v, "is_fixed", True)
                supports.append(v)
                break
    return supports


# ==============================================================================
# PHASE 3 — LOADS
# ==============================================================================

def apply_loads(form: FormDiagram, supports: List[int], params: Dict[str, float]) -> None:
    w = float(params["w"])
    sup = set(supports)
    for v in form.vertices():
        if v in sup:
            form.vertex_attributes(v, ["px", "py", "pz"], [0.0, 0.0, 0.0])
        else:
            form.vertex_attributes(v, ["px", "py", "pz"], [0.0, 0.0, -w])


# ==============================================================================
# PHASE 4 — EQUILIBRIUM (horizontal + vertical)
# ==============================================================================

def _ensure_continuous_indexing(diagram):
    # compas/compas_tna versions vary; try common methods
    for m in ("reindex", "rekey", "compact", "compact_edges"):
        if hasattr(diagram, m):
            try:
                getattr(diagram, m)()
                return
            except Exception:
                pass



def _force_set_continuous_edge_indices(force):
    """
    Ensure force edges have a continuous 'index' attribute 0..m-1.
    compas_tna ForceDiagram.ordered_edges(form) relies on this mapping.
    """
    for k, (u, v) in enumerate(force.edges()):
        e = (u, v)
        er = (v, u)

        # COMPAS has_edge expects a single edge key (tuple)
        if hasattr(force, "has_edge") and force.has_edge(e):
            force.edge_attribute(e, "index", k)
        elif hasattr(force, "has_edge") and force.has_edge(er):
            force.edge_attribute(er, "index", k)
        else:
            # If the DS treats edges as undirected, edge_attribute on (u,v) still usually works.
            try:
                force.edge_attribute(e, "index", k)
            except Exception:
                pass


""" def _debug_force_edge_index(force):
    idxs = []
    for (u, v) in force.edges():
        val = force.edge_attribute((u, v), "index")
        if isinstance(val, int):
            idxs.append(val)

    if not idxs:
        print("Force edge indices: NONE")
        return

    s = set(idxs)
    m = force.number_of_edges()
    missing = [i for i in range(m) if i not in s]

    print(f"Force edges: {m}, indexed: {len(s)}, min={min(s)}, max={max(s)}, missing={len(missing)}")
    if missing:
        print("Missing indices (first 10):", missing[:10]) 
"""

def _debug_force_edge_index(force):
    idxs = []
    for (u, v) in force.edges():
        val = force.edge_attribute((u, v), "index")
        if isinstance(val, (int, float)):
            idxs.append(int(val))

    if not idxs:
        print("Force edge indices: NONE")
        return

    s = set(idxs)
    m = force.number_of_edges()
    missing = [i for i in range(m) if i not in s]

    print(f"Force edges: {m}, indexed: {len(s)}, min={min(s)}, max={max(s)}, missing={len(missing)}")
    if missing:
        print("Missing indices (first 10):", missing[:10])

def _monkeypatch_ordered_edges(force):
    """
    Workaround for compas_tna 0.7.x bug where ForceDiagram.ordered_edges(form)
    crashes with KeyError despite valid topology.

    We replace it with a stable ordering: sort edges by their own 'index' attribute
    if present; otherwise keep current order.
    """
    def _ordered_edges(_form):
        edges = list(force.edges())
        # sort by 'index' if it exists, otherwise leave at end
        def key(e):
            idx = force.edge_attribute(e, "index")
            return int(idx) if isinstance(idx, (int, float)) else 10**18
        edges.sort(key=key)
        return edges

    force.ordered_edges = _ordered_edges

def _set_continuous_edge_indices(ds):
    """
    Set edge attribute 'index' = 0..m-1 in the current iteration order.
    Works for FormDiagram and ForceDiagram.
    """
    for k, e in enumerate(ds.edges()):
        ds.edge_attribute(e, "index", k)
        #ds.edge_attribute(e, "order", k)


def _monkeypatch_ordered_edges_by_index(ds):
    """
    Replace ds.ordered_edges(other) with a stable ordering by edge 'index'.
    This ensures both diagrams use the same ordering contract inside horizontal_nodal().
    """
    def _ordered_edges(_other):
        edges = list(ds.edges())
        edges.sort(key=lambda e: int(ds.edge_attribute(e, "index") or 10**18))
        return edges

    ds.ordered_edges = _ordered_edges

def _monkeypatch_force_ordered_edges(force):
    """
    Replace compas_tna ForceDiagram.ordered_edges(form) with a safe ordering
    that returns FORCE edges (not form edges).

    Sort by edge attribute 'index' if present; otherwise by enumeration order.
    """
    def _ordered_edges(_form):
        edges = list(force.edges())
        def key(e):
            val = force.edge_attribute(e, "index")
            return int(val) if isinstance(val, (int, float)) else 10**18
        edges.sort(key=key)
        return edges

    force.ordered_edges = _ordered_edges



def _vertical_from_zmax_safe(form, zmax):
    """
    vertical_from_zmax can return either:
    - scale (float)
    - (form, scale) tuple (some builds)
    """
    result = vertical_from_zmax(form, float(zmax))
    if isinstance(result, tuple) and len(result) == 2:
        _form, scale = result
        return float(scale)
    return float(result)

def _internal_form_edges(form):
    """
    Return ONLY internal form edges: edges with 2 adjacent faces.
    For a meshgrid, this excludes boundary edges.
    Count should match ForceDiagram.number_of_edges().
    """
    internal = []
    for e in form.edges():
        try:
            faces = form.edge_faces(e)
        except TypeError:
            u, v = e
            faces = form.edge_faces(u, v)

        # faces is typically a tuple/list of two face keys, with None on boundary
        # internal if both sides exist
        if faces and len(faces) == 2 and faces[0] is not None and faces[1] is not None:
            internal.append(e)

    return internal

    
def _rebuild_force_internal_edge_index(force):
    """
    Rebuild internal edge<->index maps used by compas_tna 0.7.x equilibrium.
    This is NOT the same as setting an 'index' attribute.
    """
    edges = list(force.edges())

    # common internal map names
    if hasattr(force, "_edge_index"):
        force._edge_index = {e: i for i, e in enumerate(edges)}
    if hasattr(force, "_index_edge"):
        force._index_edge = {i: e for i, e in enumerate(edges)}

    # some DS expose callable builders
    if hasattr(force, "reindex_edges") and callable(force.reindex_edges):
        try:
            force.reindex_edges()
        except Exception:
            pass
    if hasattr(force, "reindex") and callable(force.reindex):
        try:
            force.reindex()
        except Exception:
            pass




""" def solve_equilibrium(form: FormDiagram, params: Dict[str, float]) -> Tuple[ForceDiagram, float]:
    # Prevent edge-index mapping KeyErrors in compas_tna 0.7.x
    _ensure_continuous_indexing(form)

    force = ForceDiagram.from_formdiagram(form)
    _ensure_continuous_indexing(force)

    # If ordered_edges fails, rebuild once
    try:
        _ = force.ordered_edges(form)
    except Exception:
        force = ForceDiagram.from_formdiagram(form)
        _ensure_continuous_indexing(force)

    horizontal_nodal(form, force, alpha=float(params["alpha"]), kmax=int(params["kmax_hor"]))
    scale = float(vertical_from_zmax(form, float(params["zmax"])))
    return force, scale """

""" def solve_equilibrium(form, params):
    # Make sure form indexing is clean
    _ensure_continuous_indexing(form)

    # Build force diagram
    force = ForceDiagram.from_formdiagram(form)
    _ensure_continuous_indexing(force)

    # 🔑 CRITICAL FIX: force continuous edge indices
    _force_set_continuous_edge_indices(force)
    _monkeypatch_ordered_edges(force)

    # Optional: print diagnostics ONCE
    _debug_force_edge_index(force)

    # Verify ordered_edges works; rebuild once if needed
    try:
        _ = force.ordered_edges(form)
    except Exception as e:
        print("ordered_edges failed, rebuilding ForceDiagram:", repr(e))
        force = ForceDiagram.from_formdiagram(form)
        _ensure_continuous_indexing(force)
        _force_set_continuous_edge_indices(force)

    # Now safe to solve horizontal equilibrium
    horizontal_nodal(
        form,
        force,
        alpha=float(params["alpha"]),
        kmax=int(params["kmax_hor"])
    )

    # Vertical lifting
    scale = float(vertical_from_zmax(form, float(params["zmax"])))
    print("force edge sample:", next(iter(force.edges())))
    print("force edge attrs:", force.edge_attributes(next(iter(force.edges()))))

    return force, scale 
    """

""" def solve_equilibrium(form, params):
    # Make indexing stable on the form side
    _ensure_continuous_indexing(form)
    _set_continuous_edge_indices(form)
    _monkeypatch_ordered_edges_by_index(form)

    # Build force diagram
    force = ForceDiagram.from_formdiagram(form)
    _ensure_continuous_indexing(force)
    _set_continuous_edge_indices(force)
    _monkeypatch_ordered_edges_by_index(force)

    # Debug (optional): verify counts match
    print("FORM edges:", form.number_of_edges(), "FORCE edges:", force.number_of_edges())

    # Solve horizontal equilibrium
    horizontal_nodal(
        form,
        force,
        alpha=float(params["alpha"]),
        kmax=int(params["kmax_hor"])
    )

    # Lift vertically
    scale = _vertical_from_zmax_safe(form, params["zmax"])
    return force, scale 
    """

""" def solve_equilibrium(form, params):
    _ensure_continuous_indexing(form)

    force = ForceDiagram.from_formdiagram(form)
    _ensure_continuous_indexing(force)

    # Build internal form-edge list (must match force edge count)
    internal_edges = _internal_form_edges(form)

    print("FORM edges:", form.number_of_edges(), "FORCE edges:", force.number_of_edges(), "INTERNAL form edges:", len(internal_edges))

    if len(internal_edges) != force.number_of_edges():
        raise RuntimeError(
            f"Internal edge mismatch: internal_form={len(internal_edges)} vs force={force.number_of_edges()}. "
            "Cannot proceed."
        )

    # 🔑 Critical: override the buggy ordered_edges() to return form internal edges
    force.ordered_edges = lambda _form: internal_edges

    horizontal_nodal(
        form,
        force,
        alpha=float(params["alpha"]),
        kmax=int(params["kmax_hor"])
    )

    # vertical_from_zmax can be tuple or scalar depending on build
    result = vertical_from_zmax(form, float(params["zmax"]))
    if isinstance(result, tuple):
        _, scale = result
    else:
        scale = result

    return force, float(scale)
 """

def solve_equilibrium(form, params):
    _ensure_continuous_indexing(form)

    # build force
    force = ForceDiagram.from_formdiagram(form)
    _ensure_continuous_indexing(force)

    _rebuild_force_internal_edge_index(force)


    print("ForceDiagram class:", type(force))   
    print("Force has attrs:", [a for a in ["_edge_index", "_index_edge", "edge_index", "index_edge", "reindex_edges", "reindex"] if hasattr(force, a)])
    print("Methods with 'index' in name:", [m for m in dir(force) if "index" in m.lower()])


    # ensure force edges have a stable continuous ordering
    _set_continuous_edge_indices(force)

    # critical: bypass buggy compas_tna ordered_edges that KeyErrors
    _monkeypatch_force_ordered_edges(force)

    

    # solve
    horizontal_nodal(
        form,
        force,
        alpha=float(params["alpha"]),
        kmax=int(params["kmax_hor"])
    )

    # vertical lift (tuple-safe)
    res = vertical_from_zmax(form, float(params["zmax"]))
    if isinstance(res, tuple):
        _, scale = res
    else:
        scale = res

    return force, float(scale)


# ==============================================================================
# CLEAN VIS: ONE RHINO MESH FOR FORM
# ==============================================================================

def formdiagram_to_rhinomesh(form: FormDiagram) -> Rhino.Geometry.Mesh:
    rm = Rhino.Geometry.Mesh()
    v_index = {}

    for i, v in enumerate(form.vertices()):
        x, y, z = form.vertex_coordinates(v)
        rm.Vertices.Add(x, y, z)
        v_index[v] = i

    # faces: expect quads from meshgrid
    for fkey in form.faces():
        vs = form.face_vertices(fkey)
        if len(vs) == 4:
            rm.Faces.AddFace(v_index[vs[0]], v_index[vs[1]], v_index[vs[2]], v_index[vs[3]])
        elif len(vs) == 3:
            rm.Faces.AddFace(v_index[vs[0]], v_index[vs[1]], v_index[vs[2]])

    rm.Normals.ComputeNormals()
    rm.Compact()
    return rm


# ==============================================================================
# FORCE DIAGRAM DRAW (decimated)
# ==============================================================================

def draw_force_diagram(force: ForceDiagram, params: Dict[str, float]) -> None:
    if not params.get("draw_force_diagram", True):
        return

    layer = str(params["layer_force"])
    ox = float(params["force_offset_x"])
    oy = float(params["force_offset_y"])
    s = float(params["force_scale"])

    step = max(1, int(params.get("draw_every_nth_force_edge", 40)))

    for k, (u, v) in enumerate(force.edges()):
        if k % step != 0:
            continue
        ax, ay, _ = force.vertex_coordinates(u)
        bx, by, _ = force.vertex_coordinates(v)
        _add_line((ox + s * ax, oy + s * ay, 0.0),
                  (ox + s * bx, oy + s * by, 0.0),
                  layer,
                  color=SD.Color.FromArgb(40, 40, 40))


# ==============================================================================
# COLUMNS
# ==============================================================================

def draw_columns(form: FormDiagram, supports: List[int], params: Dict[str, float]) -> None:
    if not params.get("draw_columns", True):
        return

    layer = str(params["layer_columns"])
    h = float(params["column_height"])
    col = SD.Color.FromArgb(0, 170, 0)

    for v in supports:
        x, y, z = form.vertex_coordinates(v)
        _add_line((x, y, z), (x, y, z - h), layer, color=col)


# ==============================================================================
# 9-PIECE SEAM PATTERN PER BAY (center octagon + 8 spokes)
# ==============================================================================

def _ray_square_intersection(cx, cy, dx, dy, ox, oy, bay):
    eps = 1e-12
    hits = []

    # x=ox
    if abs(dx) > eps:
        t = (ox - cx) / dx
        y = cy + t * dy
        if t > 0 and oy - eps <= y <= oy + bay + eps:
            hits.append((t, ox, y))

    # x=ox+bay
    if abs(dx) > eps:
        t = (ox + bay - cx) / dx
        y = cy + t * dy
        if t > 0 and oy - eps <= y <= oy + bay + eps:
            hits.append((t, ox + bay, y))

    # y=oy
    if abs(dy) > eps:
        t = (oy - cy) / dy
        x = cx + t * dx
        if t > 0 and ox - eps <= x <= ox + bay + eps:
            hits.append((t, x, oy))

    # y=oy+bay
    if abs(dy) > eps:
        t = (oy + bay - cy) / dy
        x = cx + t * dx
        if t > 0 and ox - eps <= x <= ox + bay + eps:
            hits.append((t, x, oy + bay))

    hits.sort(key=lambda a: a[0])
    if not hits:
        return (cx, cy)
    _, x, y = hits[0]
    return (x, y)


def bay_seam_polylines_9piece(ox, oy, bay, theta, r_frac=0.22):
    cx = ox + 0.5 * bay
    cy = oy + 0.5 * bay
    r = r_frac * bay

    inner = []
    outer = []

    for k in range(8):
        a = theta + k * (math.pi / 4.0)
        dx = math.cos(a)
        dy = math.sin(a)
        inner.append((cx + r * dx, cy + r * dy))
        outer.append(_ray_square_intersection(cx, cy, dx, dy, ox, oy, bay))

    polylines = []
    polylines.append(inner + [inner[0]])        # inner loop (octagon)
    for k in range(8):
        polylines.append([inner[k], outer[k]])  # spokes
    return polylines


def snap_pt_to_grid_xy(x, y, dx):
    return (round(x / dx) * dx, round(y / dx) * dx)


def snap_polyline_to_grid(polyline_xy, dx):
    return [snap_pt_to_grid_xy(x, y, dx) for (x, y) in polyline_xy]


def build_vertex_xy_lookup(form: FormDiagram, dx: float):
    lut = {}
    for v in form.vertices():
        x, y, _ = form.vertex_coordinates(v)
        ix = int(round(x / dx))
        iy = int(round(y / dx))
        lut[(ix, iy)] = v
    return lut


def seam_polyline_on_surface(form: FormDiagram, seam_xy, lut, dx, sample_step=2):
    pts3 = []
    n = len(seam_xy)
    for k, (x, y) in enumerate(seam_xy):
        if sample_step > 1 and k not in (0, n - 1) and (k % sample_step) != 0:
            continue
        ix = int(round(x / dx))
        iy = int(round(y / dx))
        v = lut.get((ix, iy))
        if v is None:
            continue
        pts3.append(form.vertex_coordinates(v))
    return pts3


def draw_polyline3d(pts3, layer: str, color: SD.Color, width: int = 0):
    if len(pts3) < 2:
        return
    for i in range(len(pts3) - 1):
        _add_line(tuple(pts3[i]), tuple(pts3[i + 1]), layer, color=color, width=width)


# ==============================================================================
# FORCE PROXY + SEAM TRAFFIC (engineering proxy)
# ==============================================================================

def edge_force_mag_proxy(form: FormDiagram, u, v) -> float:
    for key in ("h", "f", "q"):
        val = form.edge_attribute((u, v), key)
        if isinstance(val, (int, float)):
            return abs(float(val))
    return 0.0


def _seg_intersect_2d(a, b, c, d) -> bool:
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    def on_seg(p, q, r):
        return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
                min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))

    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)
    eps = 1e-12

    if (o1 > eps and o2 < -eps or o1 < -eps and o2 > eps) and (o3 > eps and o4 < -eps or o3 < -eps and o4 > eps):
        return True

    if abs(o1) <= eps and on_seg(a, c, b): return True
    if abs(o2) <= eps and on_seg(a, d, b): return True
    if abs(o3) <= eps and on_seg(c, a, d): return True
    if abs(o4) <= eps and on_seg(c, b, d): return True

    return False


def seam_traffic_for_bay(form: FormDiagram, seams_xy, ox, oy, bay) -> float:
    seam_segs = []
    for pl in seams_xy:
        for i in range(len(pl) - 1):
            seam_segs.append((pl[i], pl[i + 1]))

    total = 0.0
    for (u, v) in form.edges():
        x0, y0, _ = form.vertex_coordinates(u)
        x1, y1, _ = form.vertex_coordinates(v)
        mx, my = 0.5 * (x0 + x1), 0.5 * (y0 + y1)

        if not (ox <= mx <= ox + bay and oy <= my <= oy + bay):
            continue

        a = (x0, y0)
        b = (x1, y1)
        hit = False
        for (c, d) in seam_segs:
            if _seg_intersect_2d(a, b, c, d):
                hit = True
                break
        if hit:
            total += edge_force_mag_proxy(form, u, v)

    return total


def bay_force_direction_xy(form: FormDiagram, ox, oy, bay) -> Tuple[float, float]:
    cx, cy = ox + 0.5 * bay, oy + 0.5 * bay
    v0 = min(form.vertices(), key=lambda v: (form.vertex_attribute(v, "x") - cx) ** 2 + (form.vertex_attribute(v, "y") - cy) ** 2)

    x0, y0, _ = form.vertex_coordinates(v0)
    vx, vy = 0.0, 0.0

    for nbr in form.vertex_neighbors(v0):
        x1, y1, _ = form.vertex_coordinates(nbr)
        ex, ey = (x1 - x0), (y1 - y0)
        L = (ex * ex + ey * ey) ** 0.5
        if L < 1e-12:
            continue
        ex /= L
        ey /= L

        w = edge_force_mag_proxy(form, v0, nbr)
        vx += w * ex
        vy += w * ey

    L = (vx * vx + vy * vy) ** 0.5
    if L < 1e-12:
        return (1.0, 0.0)
    return (vx / L, vy / L)


# ==============================================================================
# OPTIONAL: colored form edges by force (decimated)
# ==============================================================================

def draw_form_edges_colored(form: FormDiagram, params: Dict[str, float]) -> None:
    if not params.get("draw_form_edges_colored", False):
        return

    layer = str(params["layer_form_force"])
    step = max(1, int(params.get("draw_every_nth_colored_edge", 20)))

    # gather magnitudes (decimated sampling only)
    mags = []
    edges = []
    for k, (u, v) in enumerate(form.edges()):
        if k % step != 0:
            continue
        mags.append(edge_force_mag_proxy(form, u, v))
        edges.append((u, v))

    mmax = max(mags) if mags else 1.0
    if mmax <= 0:
        mmax = 1.0

    for (u, v), m in zip(edges, mags):
        t = m / mmax
        col = _colormap_blue_red(t)
        a = form.vertex_coordinates(u)
        b = form.vertex_coordinates(v)
        _add_line(tuple(a), tuple(b), layer, color=col)


# ==============================================================================
# MAIN
# ==============================================================================

def run(params: Dict[str, float] = None):
    params = dict(PARAMS if params is None else params)

    # layers
    layers = [
        str(params["layer_form_mesh"]),
        str(params["layer_force"]),
        str(params["layer_seams"]),
        str(params["layer_columns"]),
        str(params["layer_form_force"]),
    ]
    for L in layers:
        _ensure_layer(L)

    if params.get("clear_layers_each_run", True):
        for L in layers:
            clear_layer_objects(L)

    # 1) topology
    form = make_form(params)

    # 2) supports
    supports = apply_supports(form, params)

    # 3) loads
    apply_loads(form, supports, params)

    # 4) equilibrium
    force, scale = solve_equilibrium(form, params)


    # 5) draw: form mesh
    if params.get("draw_form_mesh", True):
        rm = formdiagram_to_rhinomesh(form)
        _add_mesh(rm, str(params["layer_form_mesh"]), color=SD.Color.FromArgb(170, 180, 190))

    # 6) columns
    draw_columns(form, supports, params)

    # 7) seams (9 per bay, rotated obliquely to local force direction)
    seam_traffic = {}
    if params.get("draw_seams", True):
        bay = float(params["bay"])
        bx = int(params["bays_x"])
        by = int(params["bays_y"])
        subdiv = int(params["subdiv_per_bay"])
        dx = bay / subdiv

        lut = build_vertex_xy_lookup(form, dx)
        ob = math.radians(float(params["oblique_offset_deg"]))
        r_frac = float(params["r_frac"])
        sample_step = int(params.get("seam_sample_step", 2))
        seam_color = SD.Color.FromArgb(220, 120, 40)

        for i in range(bx):
            for j in range(by):
                ox = i * bay
                oy = j * bay

                fx, fy = bay_force_direction_xy(form, ox, oy, bay)
                theta = math.atan2(fy, fx) + ob

                seams = bay_seam_polylines_9piece(ox, oy, bay, theta, r_frac=r_frac)
                seams = [snap_polyline_to_grid(pl, dx) for pl in seams]

                # drape seams onto surface and draw
                for pl in seams:
                    pts3 = seam_polyline_on_surface(form, pl, lut, dx, sample_step=sample_step)
                    draw_polyline3d(pts3, str(params["layer_seams"]), color=seam_color, width=0)

                # compute seam traffic proxy per bay
                t = seam_traffic_for_bay(form, seams, ox, oy, bay)
                seam_traffic[(i, j)] = t

                # label (optional, light)
                cx, cy = ox + 0.5 * bay, oy + 0.5 * bay
                # place text dot slightly above surface at nearest vertex
                ix = int(round(cx / dx)); iy = int(round(cy / dx))
                v = lut.get((ix, iy))
                if v is not None:
                    x, y, z = form.vertex_coordinates(v)
                    _add_textdot(f"{t:.2f}", (x, y, z + 0.05), str(params["layer_seams"]))

    # 8) force diagram
    draw_force_diagram(force, params)

    # 9) optional: colored form edges
    draw_form_edges_colored(form, params)

    sc.doc.Views.Redraw()

    # console summary
    print("=== FUNICULAR FLOOR (3x3) ===")
    print("subdiv_per_bay:", int(params["subdiv_per_bay"]))
    print("supports:", len(supports))
    print("vertical scale:", scale)
    if seam_traffic:
        mx = max(seam_traffic.values())
        av = sum(seam_traffic.values()) / len(seam_traffic)
        print("seam_traffic max:", mx)
        print("seam_traffic avg:", av)

    return {
        "supports": len(supports),
        "scale": scale,
        "seam_traffic": seam_traffic,
    }


if __name__ == "__main__":
    run()
