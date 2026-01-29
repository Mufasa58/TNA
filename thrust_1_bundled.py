"""
Funicular Floor System — COMPAS-TNA scripted design engine (Rhino 8, Python 3.9)

This file is self-contained and intended to be imported + executed via a Rhino "runner" script.

Core phases:
0) Setup
1) Topology: grid mesh -> FormDiagram
2) Supports at column grid intersections
3) Uniform nodal loads (pz = -w)
4) Equilibrium: horizontal_nodal + vertical_from_zmax
5) Visualization: decimated form + force diagram + seams
6) Metrics: edge-force proxy + reaction proxy + seam "traffic" proxy

Segmentation (per 7x7 bay):
- 9-piece pattern: 1 center + 4 corners + 4 mid-edge pieces
- Seam pattern is rotated to be oblique to local compression flow direction
- Computes a "joint traffic" metric: sum(|edge force|) for edges crossing seams (per bay)

Notes (academic honesty):
- TNA gives membrane thrust equilibrium, not plate bending moments.
- Seam traffic is a proxy for joint shear/bending risk; full FE shell/plate validation is a later step.
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
# Parameters (single source of truth)
# ==============================================================================

PARAMS: Dict[str, float] = {
    # Geometry
    "bay": 7.0,                 # [m]
    "bays_x": 3,
    "bays_y": 3,
    "subdiv_per_bay": 20,        # <-- requested: dx = bay/subdiv = 0.1m
    "zmax": 2.0,                # [m] target max height for lifting

    # Loads
    "w": 10.0,                  # abstract nodal load magnitude (downwards)

    # Solver
    "kmax_hor": 200,
    "alpha": 100.0,

    # Drawing
    "layer_form": "TNA_FORM",
    "layer_force": "TNA_FORCE",
    "layer_seams": "TNA_SEAMS",
    "draw_vertices": False,     # points are very heavy at high resolution
    "draw_every_nth_edge": 20,  # decimation for form edges drawing (1=draw all)
    "draw_every_nth_colored_edge": 25,  # decimation for colored edges (1=draw all)
    "draw_colored_forces": False,       # keep False initially at high resolution
    "draw_every_nth_force_edge": 20,


    "draw_scale_force": 1.0,
    "force_offset_x": 24.0,
    "force_offset_y": 0.0,

    # Seams / segmentation
    "oblique_offset_deg": 45.0,  # rotate seams away from local force direction
    "seam_draw_z": 0.02,         # plan seam draw height (small)
    "inner_radius_frac": 0.22,   # center piece size relative to bay
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
              rgb01: Tuple[float, float, float] | None = None) -> None:
    doc = sc.doc
    idx = _ensure_layer(layer)

    p0 = Rhino.Geometry.Point3d(*a)
    p1 = Rhino.Geometry.Point3d(*b)

    attr = Rhino.DocObjects.ObjectAttributes()
    attr.LayerIndex = idx

    if rgb01 is not None:
        r = int(max(0, min(255, round(255 * float(rgb01[0])))))
        g = int(max(0, min(255, round(255 * float(rgb01[1])))))
        b2 = int(max(0, min(255, round(255 * float(rgb01[2])))))
        attr.ObjectColor = SD.Color.FromArgb(r, g, b2)
        attr.ColorSource = Rhino.DocObjects.ObjectColorSource.ColorFromObject

    doc.Objects.AddLine(p0, p1, attr)


def _add_point(p: Tuple[float, float, float], layer: str) -> None:
    doc = sc.doc
    idx = _ensure_layer(layer)

    pt = Rhino.Geometry.Point3d(*p)
    attr = Rhino.DocObjects.ObjectAttributes()
    attr.LayerIndex = idx
    doc.Objects.AddPoint(pt, attr)


def _color_map_blue_red(t: float) -> Tuple[float, float, float]:
    """t in [0,1] -> (r,g,b) in [0,1], blue->red."""
    t = max(0.0, min(1.0, float(t)))
    return (t, 0.0, 1.0 - t)


def clear_layer_objects(layer_name: str) -> None:
    doc = sc.doc
    layer = doc.Layers.FindName(layer_name)
    if layer is None:
        return
    layer_index = layer.Index
    ids = [obj.Id for obj in doc.Objects if obj.Attributes.LayerIndex == layer_index]
    for gid in ids:
        doc.Objects.Delete(gid, True)


# ==============================================================================
# Phase 1 — Topology (grid mesh -> FormDiagram)
# ==============================================================================

def make_form(params: Dict[str, float]) -> FormDiagram:
    bay = float(params["bay"])
    bays_x = int(params["bays_x"])
    bays_y = int(params["bays_y"])
    subdiv = int(params["subdiv_per_bay"])

    nx = bays_x * subdiv
    ny = bays_y * subdiv

    # Regular quad mesh: nx x ny faces
    mesh = Mesh.from_meshgrid(dx=bay / subdiv, nx=nx, dy=bay / subdiv, ny=ny)
    form = FormDiagram.from_mesh(mesh)

    # Explicit z=0 initialization
    for v in form.vertices():
        x, y, _ = form.vertex_coordinates(v)
        form.vertex_attributes(v, ["x", "y", "z"], [x, y, 0.0])

    return form


# ==============================================================================
# Phase 2 — Boundary conditions (supports at column grid intersections)
# ==============================================================================

def apply_supports(form: FormDiagram, params: Dict[str, float]) -> List[int]:
    bay = float(params["bay"])
    tol = 1e-6

    col_coords = set()
    for i in range(int(params["bays_x"]) + 1):
        for j in range(int(params["bays_y"]) + 1):
            col_coords.add((i * bay, j * bay))

    supports: List[int] = []
    for v in form.vertices():
        x, y, _ = form.vertex_coordinates(v)
        for (cx, cy) in col_coords:
            if abs(x - cx) < tol and abs(y - cy) < tol:
                form.vertex_attribute(v, "is_fixed", True)
                supports.append(v)
                break

    return supports


# ==============================================================================
# Phase 3 — Loads (uniform nodal pz on free vertices)
# ==============================================================================

def apply_loads(form: FormDiagram, supports: List[int], params: Dict[str, float]) -> None:
    w = float(params["w"])

    for v in form.vertices():
        if v in supports:
            form.vertex_attributes(v, ["px", "py", "pz"], [0.0, 0.0, 0.0])
        else:
            form.vertex_attributes(v, ["px", "py", "pz"], [0.0, 0.0, -w])


# ==============================================================================
# Phase 4 — Equilibrium (horizontal reciprocity + vertical lifting)
# ==============================================================================

def _rebuild_edge_indices(diagram) -> None:
    """
    Manually rebuild continuous edge indices for the diagram.
    This ensures edge attribute '_i' is a continuous 0..n-1 sequence.
    """
    for idx, edge in enumerate(diagram.edges()):
        diagram.edge_attribute(edge, "_i", idx)


def _build_force_diagram_safe(form: FormDiagram) -> Tuple[ForceDiagram, bool]:
    """
    Build a ForceDiagram from form, ensuring continuous indexing.
    Returns (force_diagram, success_flag).
    If ordered_edges fails, returns (force_diagram, False).
    """
    # Ensure form has continuous edge indices
    _rebuild_edge_indices(form)
    
    # Create force diagram
    force = ForceDiagram.from_formdiagram(form)
    
    # Manually rebuild force diagram edge indices
    _rebuild_edge_indices(force)
    
    # Validate that ordered_edges works
    try:
        _ = force.ordered_edges(form)
        return force, True
    except (KeyError, AttributeError, IndexError) as e:
        print(f"[TNA] WARNING: ordered_edges failed: {e}")
        print("[TNA] Falling back to vertical-only equilibrium (no horizontal_nodal)")
        return force, False


def solve_equilibrium(form: FormDiagram, params: Dict[str, float]) -> Tuple[ForceDiagram, float]:
    # Build force diagram
    force, can_do_horizontal = _build_force_diagram_safe(form)

    # Only run horizontal_nodal if ordered_edges works
    if can_do_horizontal:
        try:
            horizontal_nodal(
                form,
                force,
                alpha=float(params["alpha"]),
                kmax=int(params["kmax_hor"]),
            )
        except Exception as e:
            print(f"[TNA] horizontal_nodal failed: {e}")
            print("[TNA] Continuing with vertical equilibrium only...")
    else:
        # Initialize default force densities for vertical_from_zmax
        for edge in form.edges():
            if form.edge_attribute(edge, "q") is None:
                form.edge_attribute(edge, "q", 1.0)

    # vertical_from_zmax returns (form, scale) tuple
    result = vertical_from_zmax(form, float(params["zmax"]))
    if isinstance(result, tuple):
        _, scale = result
    else:
        scale = float(result)
    
    return force, scale


# ==============================================================================
# Visualization — form / force
# ==============================================================================

def draw_form(form: FormDiagram, params: Dict[str, float]) -> None:
    layer = str(params["layer_form"])
    draw_pts = bool(params.get("draw_vertices", False))
    step = max(1, int(params.get("draw_every_nth_edge", 1)))

    if draw_pts:
        for v in form.vertices():
            x, y, z = form.vertex_coordinates(v)
            _add_point((x, y, z), layer)

    for k, (u, v) in enumerate(form.edges()):
        if k % step != 0:
            continue
        a = form.vertex_coordinates(u)
        b = form.vertex_coordinates(v)
        _add_line(tuple(a), tuple(b), layer)


""" def draw_force(force: ForceDiagram, params: Dict[str, float]) -> None:
    layer = str(params["layer_force"])
    ox = float(params["force_offset_x"])
    oy = float(params["force_offset_y"])
    s = float(params["draw_scale_force"])

    for u, v in force.edges():
        ax, ay, _ = force.vertex_coordinates(u)
        bx, by, _ = force.vertex_coordinates(v)
        a = (ox + s * ax, oy + s * ay, 0.0)
        b = (ox + s * bx, oy + s * by, 0.0)
        _add_line(a, b, layer)
 """

def draw_force(force: ForceDiagram, params: Dict[str, float]) -> None:
    layer = str(params["layer_force"])
    ox = float(params["force_offset_x"])
    oy = float(params["force_offset_y"])
    s = float(params["draw_scale_force"])

    step = int(params.get("draw_every_nth_force_edge", 1))
    step = max(1, step)

    for k, (u, v) in enumerate(force.edges()):
        if k % step != 0:
            continue
        ax, ay, _ = force.vertex_coordinates(u)
        bx, by, _ = force.vertex_coordinates(v)
        a = (ox + s * ax, oy + s * ay, 0.0)
        b = (ox + s * bx, oy + s * by, 0.0)
        _add_line(a, b, layer)


# ==============================================================================
# Edge force proxy + colored drawing (optional)
# ==============================================================================

def _edge_force_magnitude(form: FormDiagram, u: int, v: int) -> float:
    for key in ("h", "f"):
        val = form.edge_attribute((u, v), key)
        if isinstance(val, (int, float)):
            return abs(float(val))
    q = form.edge_attribute((u, v), "q")
    if isinstance(q, (int, float)):
        return abs(float(q))
    return 0.0


def draw_form_colored_by_force(form: FormDiagram, params: Dict[str, float]) -> None:
    """
    Decimated colored edges by |force| proxy.
    WARNING: still heavy at high resolution; keep draw_colored_forces=False until stable.
    """
    base_layer = str(params["layer_form"]) + "_FORCECOLOR"
    step = max(1, int(params.get("draw_every_nth_colored_edge", 1)))

    mags = []
    edges = []
    for u, v in form.edges():
        mags.append(_edge_force_magnitude(form, u, v))
        edges.append((u, v))

    mmax = max(mags) if mags else 1.0
    if mmax <= 0.0:
        mmax = 1.0

    for k, ((u, v), m) in enumerate(zip(edges, mags)):
        if k % step != 0:
            continue
        t = m / mmax
        rgb = _color_map_blue_red(t)
        a = form.vertex_coordinates(u)
        b = form.vertex_coordinates(v)
        _add_line(tuple(a), tuple(b), base_layer, rgb01=rgb)


# ==============================================================================
# Segmentation — 9-piece seams per bay (force-aware rotation)
# ==============================================================================

def _ray_square_intersection(cx, cy, dx, dy, ox, oy, bay):
    """Intersect ray from (cx,cy) in direction (dx,dy) with square [ox..ox+bay]x[oy..oy+bay]."""
    eps = 1e-12
    t_candidates = []

    if abs(dx) > eps:
        t = (ox - cx) / dx
        y = cy + t * dy
        if t > 0 and oy - eps <= y <= oy + bay + eps:
            t_candidates.append((t, ox, y))

        t = (ox + bay - cx) / dx
        y = cy + t * dy
        if t > 0 and oy - eps <= y <= oy + bay + eps:
            t_candidates.append((t, ox + bay, y))

    if abs(dy) > eps:
        t = (oy - cy) / dy
        x = cx + t * dx
        if t > 0 and ox - eps <= x <= ox + bay + eps:
            t_candidates.append((t, x, oy))

        t = (oy + bay - cy) / dy
        x = cx + t * dx
        if t > 0 and ox - eps <= x <= ox + bay + eps:
            t_candidates.append((t, x, oy + bay))

    t_candidates.sort(key=lambda a: a[0])
    if not t_candidates:
        return (cx, cy)
    _, x, y = t_candidates[0]
    return (x, y)


def bay_seam_polylines_9piece(ox, oy, bay, theta_rad, r_frac=0.22):
    """
    Seam polylines for one bay:
    - inner octagon loop (center piece boundary)
    - 8 spokes from inner octagon vertices to perimeter points
    """
    cx = ox + 0.5 * bay
    cy = oy + 0.5 * bay
    r = float(r_frac) * bay

    inner = []
    outer = []

    for k in range(8):
        a = theta_rad + k * (math.pi / 4.0)
        dx = math.cos(a)
        dy = math.sin(a)

        inner.append((cx + r * dx, cy + r * dy))
        outer.append(_ray_square_intersection(cx, cy, dx, dy, ox, oy, bay))

    polylines = []
    polylines.append(inner + [inner[0]])  # inner loop
    for k in range(8):
        polylines.append([inner[k], outer[k]])  # spokes

    return polylines


def snap_pt_to_grid_xy(x, y, dx):
    sx = round(x / dx) * dx
    sy = round(y / dx) * dx
    return (sx, sy)


def snap_polyline_to_grid(polyline_xy, dx):
    return [snap_pt_to_grid_xy(x, y, dx) for (x, y) in polyline_xy]


def draw_seam_polylines_in_rhino(seam_polylines_xy, layer, z=0.0):
    for pl in seam_polylines_xy:
        for i in range(len(pl) - 1):
            x0, y0 = pl[i]
            x1, y1 = pl[i + 1]
            _add_line((x0, y0, z), (x1, y1, z), layer)


def bay_force_direction_xy(form, ox, oy, bay):
    """Dominant XY direction near bay center using weighted sum of incident edge directions."""
    cx, cy = ox + 0.5 * bay, oy + 0.5 * bay

    v0 = min(
        form.vertices(),
        key=lambda v: (form.vertex_attribute(v, "x") - cx) ** 2 + (form.vertex_attribute(v, "y") - cy) ** 2
    )

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

        # Try force keys in order
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
# Seam traffic metric (proxy for joint demand)
# ==============================================================================

def _seg_intersect_2d(a, b, c, d):
    """2D segment intersection (including touching). a,b,c,d are (x,y)."""
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

    if (o1 > eps and o2 < -eps or o1 < -eps and o2 > eps) and (o3 > eps and o4 < -eps or o3 < -eps and o4 > eps):
        return True

    if abs(o1) <= eps and on_seg(a, c, b): return True
    if abs(o2) <= eps and on_seg(a, d, b): return True
    if abs(o3) <= eps and on_seg(c, a, d): return True
    if abs(o4) <= eps and on_seg(c, b, d): return True

    return False


def seam_traffic_for_bay(form, seam_polylines_xy, ox, oy, bay):
    """
    Sum |force| of form edges that intersect any seam segment.
    Only edges whose midpoint lies within the bay are considered.
    """
    seam_segs = []
    for pl in seam_polylines_xy:
        for i in range(len(pl) - 1):
            seam_segs.append((pl[i], pl[i + 1]))

    total = 0.0
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
            total += _edge_force_magnitude(form, u, v)

    return total


# ==============================================================================
# Metrics
# ==============================================================================

def compute_metrics(form: FormDiagram, supports: List[int]) -> Dict[str, float]:
    edge_mags = [_edge_force_magnitude(form, u, v) for (u, v) in form.edges()]
    if not edge_mags:
        edge_mags = [0.0]

    # Reaction proxy (if 'rz' exists, use it; else load-balance proxy)
    rz_vals = []
    for v in supports:
        rz = form.vertex_attribute(v, "rz")
        if isinstance(rz, (int, float)):
            rz_vals.append(float(rz))

    if rz_vals:
        rmax = max(abs(r) for r in rz_vals)
        ravg = sum(abs(r) for r in rz_vals) / len(rz_vals)
    else:
        total_pz = 0.0
        for v in form.vertices():
            pz = form.vertex_attribute(v, "pz") or 0.0
            total_pz += float(pz)
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

def run(params: Dict[str, float] = None):
    params = dict(PARAMS if params is None else params)

    # Phase 0 — Layers: ensure they exist
    _ensure_layer(str(params["layer_form"]))
    _ensure_layer(str(params["layer_force"]))
    _ensure_layer(str(params["layer_seams"]))
    
    # Clear existing geometry from all TNA layers before drawing new geometry
    clear_layer_objects(str(params["layer_form"]))
    clear_layer_objects(str(params["layer_force"]))
    clear_layer_objects(str(params["layer_seams"]))
    clear_layer_objects(str(params["layer_form"]) + "_FORCECOLOR")

    # Phase 1 — Topology
    form = make_form(params)

    # Phase 2 — Supports
    supports = apply_supports(form, params)

    # Phase 3 — Loads
    apply_loads(form, supports, params)

    # Phase 4 — Equilibrium
    force, scale = solve_equilibrium(form, params)

    # Phase 6A — Seams + seam traffic (per bay)
    bay = float(params["bay"])
    bx = int(params["bays_x"])
    by = int(params["bays_y"])
    subdiv = int(params["subdiv_per_bay"])
    dx = bay / subdiv  # grid spacing
    ob = math.radians(float(params["oblique_offset_deg"]))
    seam_z = float(params.get("seam_draw_z", 0.02))
    r_frac = float(params.get("inner_radius_frac", 0.22))

    traffic_sum = 0.0
    traffic_max = 0.0

    for i in range(bx):
        for j in range(by):
            ox = i * bay
            oy = j * bay

            fx, fy = bay_force_direction_xy(form, ox, oy, bay)
            theta = math.atan2(fy, fx) + ob

            seams = bay_seam_polylines_9piece(ox, oy, bay, theta, r_frac=r_frac)
            seams = [snap_polyline_to_grid(pl, dx) for pl in seams]

            draw_seam_polylines_in_rhino(seams, str(params["layer_seams"]), z=seam_z)

            t = seam_traffic_for_bay(form, seams, ox, oy, bay)
            traffic_sum += t
            traffic_max = max(traffic_max, t)

    # Phase 5 — Draw (decimated form + force)
    draw_form(form, params)
    draw_force(force, params)

    if bool(params.get("draw_colored_forces", False)):
        draw_form_colored_by_force(form, params)

    # Phase 6B — Metrics
    metrics = compute_metrics(form, supports)

    sc.doc.Views.Redraw()

    print("=== TNA Funicular Floor: results ===")
    print(f"supports (count): {len(supports)}")
    print(f"vertical scale (from zmax): {scale:.6g}")
    print(f"max edge |force| (proxy): {metrics['edge_force_max']:.6g}")
    print(f"avg edge |force| (proxy): {metrics['edge_force_avg']:.6g}")
    print(f"max |reaction| (proxy):   {metrics['reaction_max_abs']:.6g}")
    print(f"avg |reaction| (proxy):   {metrics['reaction_avg_abs']:.6g}")
    print("=== Seam / Joint Metrics (proxy) ===")
    print(f"Total seam traffic (sum over 9 bays): {traffic_sum:.6g}")
    print(f"Max seam traffic (worst bay):         {traffic_max:.6g}")

    return {
        "supports": len(supports),
        "scale": scale,
        "metrics": metrics,
        "seam_traffic_sum": traffic_sum,
        "seam_traffic_max": traffic_max,
        "dx": dx,
    }


if __name__ == "__main__":
    run()
