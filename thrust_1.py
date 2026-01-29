"""
Funicular Floor System — COMPAS-TNA scripted exemplar (Rhino 8, Python 3.9)

Implements:
- FormDiagram creation (grid mesh -> FormDiagram)
- Column supports (at bay intersections)
- Uniform nodal vertical loads (pz = -w)
- ForceDiagram creation (dual)
- Horizontal equilibrium (TNA reciprocity)
- Vertical equilibrium ("lifting") via vertical_from_zmax
- Rhino drawing: form + force diagrams
- Edge force extraction + simple efficiency metrics
- Edge coloring by |force| magnitude

Docs anchors (API + concepts):
- ForceDiagram.from_formdiagram :contentReference[oaicite:0]{index=0}
- horizontal_nodal(form, force, alpha=100, kmax=100, ...) :contentReference[oaicite:1]{index=1}
- vertical_from_q(...) meaning of q/h/l :contentReference[oaicite:2]{index=2}
- TNA overview: form/force diagram roles :contentReference[oaicite:3]{index=3}
- Mesh.from_meshgrid(dx, nx, dy=None, ny=None) :contentReference[oaicite:4]{index=4}
"""

from __future__ import annotations

import math
from typing import Dict, Tuple, List

import Rhino
import scriptcontext as sc

from compas.datastructures import Mesh
from compas_tna.diagrams import FormDiagram, ForceDiagram
from compas_tna.equilibrium import horizontal_nodal, vertical_from_zmax


# ==============================================================================
# Parameters (single source of truth)
# ==============================================================================

PARAMS: Dict[str, float] = {
    # Geometry
    "bay": 7.0,                 # [m] bay size in X and Y
    "bays_x": 3,                # 3 bays -> total span = 21m
    "bays_y": 3,
    "subdiv_per_bay": 8,        # refine mesh so there are free nodes between columns
    "zmax": 2.0,                # [m] target max height (lift) for the thrust network

    # Loads
    "w": 10.0,                  # abstract nodal load magnitude (downwards), units arbitrary

    # Solver
    "kmax_hor": 200,
    "alpha": 100.0,             # alpha=100 keeps the *form* XY mostly fixed during parallelisation :contentReference[oaicite:5]{index=5}

    # Drawing
    "draw_scale_form": 1.0,     # Rhino units assumed meters
    "draw_scale_force": 1.0,    # force diagram drawn "as is" then scaled
    "force_offset_x": 24.0,     # move force diagram to the right in Rhino view
    "force_offset_y": 0.0,
    "layer_form": "TNA_FORM",
    "layer_force": "TNA_FORCE",
}


# ==============================================================================
# Rhino helpers
# ==============================================================================

def _ensure_layer(name: str) -> int:
    """Create layer if missing. Return layer index."""
    doc = sc.doc
    layer = doc.Layers.FindName(name)
    if layer is None:
        layer = Rhino.DocObjects.Layer()
        layer.Name = name
        idx = doc.Layers.Add(layer)
        return idx
    return layer.Index


def _add_line(a: Tuple[float, float, float],
              b: Tuple[float, float, float],
              layer: str,
              color: Rhino.Display.Color4f | None = None) -> Rhino.Geometry.Line:
    doc = sc.doc
    idx = _ensure_layer(layer)

    p0 = Rhino.Geometry.Point3d(*a)
    p1 = Rhino.Geometry.Point3d(*b)
    line = Rhino.Geometry.Line(p0, p1)

    attr = Rhino.DocObjects.ObjectAttributes()
    attr.LayerIndex = idx
    if color is not None:
        # Convert Color4f -> System.Drawing.Color
        c = Rhino.Display.Color4f(color)
        sd = Rhino.Display.Color4f.ToArgbColor(c)  # stable conversion helper in RhinoCommon
        attr.ObjectColor = sd
        attr.ColorSource = Rhino.DocObjects.ObjectColorSource.ColorFromObject

    doc.Objects.AddLine(line, attr)
    return line


def _add_point(p: Tuple[float, float, float], layer: str) -> None:
    doc = sc.doc
    idx = _ensure_layer(layer)

    pt = Rhino.Geometry.Point3d(*p)
    attr = Rhino.DocObjects.ObjectAttributes()
    attr.LayerIndex = idx
    doc.Objects.AddPoint(pt, attr)


def _color_map_blue_red(t: float) -> Rhino.Display.Color4f:
    """t in [0,1] -> blue->red."""
    t = max(0.0, min(1.0, t))
    r = float(t)
    g = 0.0
    b = float(1.0 - t)
    return Rhino.Display.Color4f(r, g, b, 1.0)


# ==============================================================================
# Phase 1 — Topology (grid mesh -> FormDiagram)
# ==============================================================================

def make_form(params: Dict[str, float]) -> FormDiagram:
    bay = float(params["bay"])
    bays_x = int(params["bays_x"])
    bays_y = int(params["bays_y"])
    subdiv = int(params["subdiv_per_bay"])

    # Mesh resolution (faces):
    # 3 bays, each subdivided -> gives interior nodes between columns
    nx = bays_x * subdiv
    ny = bays_y * subdiv

    # Regular quad mesh: (nx faces) x (ny faces) :contentReference[oaicite:6]{index=6}
    mesh = Mesh.from_meshgrid(dx=bay / subdiv, nx=nx, dy=bay / subdiv, ny=ny)

    # FormDiagram stores XY from mesh, Z starts at 0 :contentReference[oaicite:7]{index=7}
    form = FormDiagram.from_mesh(mesh)

    # Move/scale into a 0..21 range (meshgrid starts at origin already; dx sets spacing)
    # Ensure explicit z=0
    for v in form.vertices():
        x, y, _ = form.vertex_coordinates(v)
        form.vertex_attributes(v, ["x", "y", "z"], [x, y, 0.0])

    return form

def _ray_square_intersection(cx, cy, dx, dy, ox, oy, bay):
    """
    Intersect ray from (cx,cy) in direction (dx,dy) with square [ox,ox+bay]x[oy,oy+bay].
    Returns intersection point on perimeter.
    """
    eps = 1e-12
    t_candidates = []

    # x = ox
    if abs(dx) > eps:
        t = (ox - cx) / dx
        y = cy + t * dy
        if t > 0 and oy - eps <= y <= oy + bay + eps:
            t_candidates.append((t, ox, y))

    # x = ox+bay
    if abs(dx) > eps:
        t = (ox + bay - cx) / dx
        y = cy + t * dy
        if t > 0 and oy - eps <= y <= oy + bay + eps:
            t_candidates.append((t, ox + bay, y))

    # y = oy
    if abs(dy) > eps:
        t = (oy - cy) / dy
        x = cx + t * dx
        if t > 0 and ox - eps <= x <= ox + bay + eps:
            t_candidates.append((t, x, oy))

    # y = oy+bay
    if abs(dy) > eps:
        t = (oy + bay - cy) / dy
        x = cx + t * dx
        if t > 0 and ox - eps <= x <= ox + bay + eps:
            t_candidates.append((t, x, oy + bay))

    # pick closest positive intersection
    t_candidates.sort(key=lambda a: a[0])
    if not t_candidates:
        # fallback: no hit (shouldn't happen)
        return (cx, cy)
    _, x, y = t_candidates[0]
    return (x, y)


def bay_seam_polylines_9piece(ox, oy, bay, theta_rad, r_frac=0.22):
    """
    Returns seam polylines (as list of 2D point lists) for one bay.
    Pattern:
      - inner octagon (center piece boundary)
      - 8 spokes from inner octagon vertices to perimeter points
    """
    cx = ox + 0.5 * bay
    cy = oy + 0.5 * bay
    r = r_frac * bay

    inner = []
    outer = []

    # 8 directions at 45deg increments, rotated by theta
    for k in range(8):
        a = theta_rad + k * (math.pi / 4.0)
        dx = math.cos(a)
        dy = math.sin(a)

        ix = cx + r * dx
        iy = cy + r * dy
        inner.append((ix, iy))

        px, py = _ray_square_intersection(cx, cy, dx, dy, ox, oy, bay)
        outer.append((px, py))

    polylines = []

    # inner octagon loop
    polylines.append(inner + [inner[0]])

    # 8 spokes: inner[k] -> outer[k]
    for k in range(8):
        polylines.append([inner[k], outer[k]])

    return polylines


# ==============================================================================
# Phase 2 — Boundary conditions (supports at column grid intersections)
# ==============================================================================

def apply_supports(form: FormDiagram, params: Dict[str, float]) -> List[int]:
    bay = float(params["bay"])
    subdiv = int(params["subdiv_per_bay"])
    tol = 1e-6

    # Column locations are at multiples of 7m in X and Y:
    # (0,7,14,21) x (0,7,14,21)
    col_coords = set()
    for i in range(int(params["bays_x"]) + 1):
        for j in range(int(params["bays_y"]) + 1):
            col_coords.add((i * bay, j * bay))

    supports: List[int] = []
    for v in form.vertices():
        x, y, _ = form.vertex_coordinates(v)
        # snap check: x,y should be multiples of (bay/subdiv); columns are multiples of bay
        for (cx, cy) in col_coords:
            if abs(x - cx) < tol and abs(y - cy) < tol:
                form.vertex_attribute(v, "is_fixed", True)
                supports.append(v)
                break

    return supports

def snap_pt_to_grid_xy(x, y, dx):
    sx = round(x / dx) * dx
    sy = round(y / dx) * dx
    return sx, sy

def snap_polyline_to_grid(polyline_xy, dx):
    return [snap_pt_to_grid_xy(x, y, dx) for (x, y) in polyline_xy]

def bay_force_direction_xy(form, ox, oy, bay):
    """
    Dominant XY direction near bay center using weighted sum of incident edge directions.
    """
    cx, cy = ox + 0.5 * bay, oy + 0.5 * bay

    # pick nearest vertex to bay center
    v0 = min(
        form.vertices(),
        key=lambda v: (form.vertex_attribute(v, "x") - cx) ** 2 + (form.vertex_attribute(v, "y") - cy) ** 2
    )

    vx, vy = 0.0, 0.0
    x0, y0, _ = form.vertex_coordinates(v0)

    for nbr in form.vertex_neighbors(v0):
        x1, y1, _ = form.vertex_coordinates(nbr)
        ex, ey = (x1 - x0), (y1 - y0)
        L = (ex * ex + ey * ey) ** 0.5
        if L < 1e-12:
            continue
        ex /= L
        ey /= L

        # try common edge force keys
        h = form.edge_attribute((v0, nbr), "h")
        if h is None:
            h = form.edge_attribute((v0, nbr), "f")
        if h is None:
            h = form.edge_attribute((v0, nbr), "q")
        w = abs(float(h or 0.0))

        vx += w * ex
        vy += w * ey

    L = (vx * vx + vy * vy) ** 0.5
    if L < 1e-12:
        return (1.0, 0.0)
    return (vx / L, vy / L)


# ==============================================================================
# Phase 3 — Loads (uniform nodal pz on free vertices)
# ==============================================================================

def apply_loads(form: FormDiagram, supports: List[int], params: Dict[str, float]) -> None:
    w = float(params["w"])

    # TNA assumes vertical loads at nodes (px,py,pz); here only pz is used :contentReference[oaicite:8]{index=8}
    for v in form.vertices():
        if v in supports:
            form.vertex_attributes(v, ["px", "py", "pz"], [0.0, 0.0, 0.0])
        else:
            form.vertex_attributes(v, ["px", "py", "pz"], [0.0, 0.0, -w])

def _seg_intersect_2d(a, b, c, d):
    """
    Proper 2D segment intersection test (including touching).
    a,b,c,d are (x,y)
    """
    def orient(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1]) - (q[1]-p[1])*(r[0]-p[0])

    def on_seg(p, q, r):
        return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
                min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))

    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)

    eps = 1e-12
    # general case
    if (o1 > eps and o2 < -eps or o1 < -eps and o2 > eps) and (o3 > eps and o4 < -eps or o3 < -eps and o4 > eps):
        return True

    # collinear / touching
    if abs(o1) <= eps and on_seg(a, c, b): return True
    if abs(o2) <= eps and on_seg(a, d, b): return True
    if abs(o3) <= eps and on_seg(c, a, d): return True
    if abs(o4) <= eps and on_seg(c, b, d): return True

    return False


def edge_force_mag_proxy(form, u, v):
    for key in ("h", "f", "q"):
        val = form.edge_attribute((u, v), key)
        if isinstance(val, (int, float)):
            return abs(float(val))
    return 0.0


def seam_traffic_for_bay(form, seam_polylines_xy, ox, oy, bay):
    """
    Sum |force| of form edges that intersect any seam segment.
    Only consider edges whose midpoint lies inside the bay.
    """
    total = 0.0

    # flatten seam segments
    seam_segs = []
    for pl in seam_polylines_xy:
        for i in range(len(pl) - 1):
            seam_segs.append((pl[i], pl[i + 1]))

    for u, v in form.edges():
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

# ==============================================================================
# Phase 4 — Equilibrium (horizontal reciprocity + vertical lifting)
# ==============================================================================

def solve_equilibrium(form: FormDiagram, params: Dict[str, float]) -> Tuple[ForceDiagram, float]:
    # Force diagram is the dual of the form diagram :contentReference[oaicite:9]{index=9}
    force = ForceDiagram.from_formdiagram(form)

    # Horizontal equilibrium: parallelise corresponding edges (reciprocity) :contentReference[oaicite:10]{index=10}
    horizontal_nodal(
        form,
        force,
        alpha=float(params["alpha"]),
        kmax=int(params["kmax_hor"]),
    )

    # Vertical equilibrium ("lifting"): pick a scale that gives desired zmax :contentReference[oaicite:11]{index=11}
    scale = vertical_from_zmax(form, float(params["zmax"]))

    return force, float(scale)


# ==============================================================================
# Phase 5 — Visualization (Rhino draw + color edges by force magnitude)
# ==============================================================================

def draw_form(form: FormDiagram, params: Dict[str, float]) -> None:
    layer = str(params["layer_form"])

    # Draw vertices (optional, light)
    for v in form.vertices():
        x, y, z = form.vertex_coordinates(v)
        _add_point((x, y, z), layer)

    # Draw edges
    for u, v in form.edges():
        a = form.vertex_coordinates(u)
        b = form.vertex_coordinates(v)
        _add_line(tuple(a), tuple(b), layer)


def _edge_force_magnitude(form: FormDiagram, u: int, v: int) -> float:
    """
    Robustly extract a force magnitude proxy from edge attributes.

    In compas_tna, horizontal forces and/or force densities relate through:
    q = scale * (l_force / l_form) and h = q * l_form :contentReference[oaicite:12]{index=12}

    We try common keys in order: 'h', 'f', 'q' (fallback uses |q|).
    """
    for key in ("h", "f"):
        val = form.edge_attribute((u, v), key)
        if isinstance(val, (int, float)) and not math.isnan(float(val)):
            return abs(float(val))

    q = form.edge_attribute((u, v), "q")
    if isinstance(q, (int, float)) and not math.isnan(float(q)):
        return abs(float(q))

    return 0.0


def draw_form_colored_by_force(form: FormDiagram, params: Dict[str, float]) -> None:
    layer = str(params["layer_form"])

    # collect magnitudes
    mags = []
    edges = []
    for u, v in form.edges():
        m = _edge_force_magnitude(form, u, v)
        mags.append(m)
        edges.append((u, v))

    mmax = max(mags) if mags else 1.0
    if mmax <= 0.0:
        mmax = 1.0

    # redraw edges colored (does not delete previous geometry; use a fresh layer name if you want)
    colored_layer = layer + "_FORCECOLOR"
    for (u, v), m in zip(edges, mags):
        t = m / mmax
        col = _color_map_blue_red(t)
        a = form.vertex_coordinates(u)
        b = form.vertex_coordinates(v)
        _add_line(tuple(a), tuple(b), colored_layer, color=col)


def draw_force(force: ForceDiagram, params: Dict[str, float]) -> None:
    layer = str(params["layer_force"])
    ox = float(params["force_offset_x"])
    oy = float(params["force_offset_y"])
    s = float(params["draw_scale_force"])

    # The force diagram is 2D (XY); draw it next to the form diagram
    for u, v in force.edges():
        ax, ay, _ = force.vertex_coordinates(u)
        bx, by, _ = force.vertex_coordinates(v)
        a = (ox + s * ax, oy + s * ay, 0.0)
        b = (ox + s * bx, oy + s * by, 0.0)
        _add_line(a, b, layer)


# ==============================================================================
# Phase 6 — Metrics (forces + reactions proxy)
# ==============================================================================

def compute_metrics(form: FormDiagram, supports: List[int]) -> Dict[str, float]:
    edge_mags = [_edge_force_magnitude(form, u, v) for (u, v) in form.edges()]
    if not edge_mags:
        edge_mags = [0.0]

    # Reactions:
    # If compas_tna stored reactions, they'd be vertex attributes (varies by version).
    # We attempt 'rz' and fall back to load-balance proxy (sum of applied pz / number of supports).
    rz_vals = []
    for v in supports:
        rz = form.vertex_attribute(v, "rz")
        if isinstance(rz, (int, float)) and not math.isnan(float(rz)):
            rz_vals.append(float(rz))

    if rz_vals:
        rmax = max(abs(r) for r in rz_vals)
        ravg = sum(abs(r) for r in rz_vals) / len(rz_vals)
    else:
        total_pz = 0.0
        for v in form.vertices():
            pz = form.vertex_attribute(v, "pz") or 0.0
            total_pz += float(pz)
        # support reaction sum should balance -total_pz
        per_support = abs(total_pz) / max(1, len(supports))
        rmax = per_support
        ravg = per_support

    return {
        "edge_force_max": max(edge_mags),
        "edge_force_avg": sum(edge_mags) / len(edge_mags),
        "reaction_max_abs": rmax,
        "reaction_avg_abs": ravg,
    }


# ==============================================================================
# Main entrypoint
# ==============================================================================

def run(params: Dict[str, float] = None) -> None:
    params = dict(PARAMS if params is None else params)

    # Phase 0 — Setup: ensure layers exist
    _ensure_layer(str(params["layer_form"]))
    _ensure_layer(str(params["layer_force"]))

    # Phase 1 — Topology
    form = make_form(params)

    # Phase 2 — Supports
    supports = apply_supports(form, params)

    # Phase 3 — Loads
    apply_loads(form, supports, params)

    # Phase 4 — Equilibrium
    force, scale = solve_equilibrium(form, params)

    # Phase 5 — Draw
    draw_form(form, params)
    draw_force(force, params)
    draw_form_colored_by_force(form, params)

    # Phase 6 — Metrics
    metrics = compute_metrics(form, supports)

    sc.doc.Views.Redraw()

    print("=== TNA Funicular Floor: results ===")
    print(f"supports (count): {len(supports)}")
    print(f"vertical scale (from zmax): {scale:.6g}")
    print(f"max edge |force| (proxy): {metrics['edge_force_max']:.6g}")
    print(f"avg edge |force| (proxy): {metrics['edge_force_avg']:.6g}")
    print(f"max |reaction| (proxy):   {metrics['reaction_max_abs']:.6g}")
    print(f"avg |reaction| (proxy):   {metrics['reaction_avg_abs']:.6g}")


# If you press "Run" in Rhino Script Editor, this will execute.
if __name__ == "__main__":
    run()
