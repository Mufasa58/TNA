#! python3
# venv: rvdev

import math
import Rhino
import scriptcontext as sc
import System
from System.Drawing import Color

from compas_tna.diagrams import FormDiagram
from compas_tna.equilibrium import vertical_from_zmax


# -----------------------------
# PARAMETERS (edit these)
# -----------------------------
BAY = 7.0
NBAYS = 3
L = BAY * NBAYS            # 21m
NX = 24                    # mesh resolution in x (increase for smoother)
NY = 24                    # mesh resolution in y
ZMAX = 3.0                 # target rise (m) -> bigger = “deeper” funicular
DENSITY = 1.0              # load density proxy (relative)
KMAX = 200
XTOL = 0.01
RTOL = 0.001

DRAW_LAYER = "TNA::Form"
REACTION_LAYER = "TNA::Reactions"


# -----------------------------
# RHINO helpers
# -----------------------------
def ensure_layer(name, color):
    idx = sc.doc.Layers.FindByFullPath(name, True)
    if idx >= 0:
        return idx
    layer = Rhino.DocObjects.Layer()
    layer.Name = name
    layer.Color = color
    return sc.doc.Layers.Add(layer)

def draw_lines(lines, layer_name, color):
    layer_idx = ensure_layer(layer_name, color)
    attr = Rhino.DocObjects.ObjectAttributes()
    attr.LayerIndex = layer_idx
    ids = []
    for a, b in lines:
        ids.append(sc.doc.Objects.AddLine(a, b, attr))
    return ids

def draw_arrows(arrows, layer_name, color):
    layer_idx = ensure_layer(layer_name, color)
    attr = Rhino.DocObjects.ObjectAttributes()
    attr.LayerIndex = layer_idx
    ids = []
    for a, b in arrows:
        ids.append(sc.doc.Objects.AddLine(a, b, attr))
    return ids


# -----------------------------
# 1) Build form diagram mesh
# -----------------------------
# Meshgrid uses dx, dy, nx, ny as step sizes + counts; easiest is:
dx = L / NX
dy = L / NY

form = FormDiagram.from_meshgrid(dx=dx, nx=NX, dy=dy, ny=NY)

# Move to world coords so it spans [0..L] in x and y
# (meshgrid already starts at origin, so nothing else needed)


# -----------------------------
# 2) Define supports (4x4 columns)
# -----------------------------
col_coords = set()
for i in range(NBAYS + 1):
    for j in range(NBAYS + 1):
        col_coords.add((round(i * BAY, 6), round(j * BAY, 6)))

support_count = 0
for v in form.vertices():
    x, y, z = form.vertex_coordinates(v)
    key = (round(x, 6), round(y, 6))
    if key in col_coords:
        form.vertex_attribute(v, "is_support", True)
        support_count += 1

print("Supports set:", support_count, "(expected 16)")

# -----------------------------
# 3) Solve funicular form (vertical)
# -----------------------------
# This computes a funicular shape under uniform density with target rise ZMAX
form, scale = vertical_from_zmax(
    form=form,
    zmax=ZMAX,
    kmax=KMAX,
    xtol=XTOL,
    rtol=RTOL,
    density=DENSITY,
    display=False
)

print("Solved. scale:", scale)


# -----------------------------
# 4) Compute support reactions (proxy) + objective
# -----------------------------
# COMPAS-TNA stores loads implicitly; a simple robust proxy is:
# reaction = -sum(edge forces * unit vectors) for edges incident at support
# We'll compute from force densities if available; otherwise use geometry-only proxy.
#
# In vertical_from_zmax results, edges usually have 'q' force density.
# Force in edge ~ q * length. Direction is along edge.

def vec(p, q):
    return (q.X - p.X, q.Y - p.Y, q.Z - p.Z)

def length(v):
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def unit(v):
    l = length(v)
    if l < 1e-12:
        return (0.0, 0.0, 0.0)
    return (v[0]/l, v[1]/l, v[2]/l)

reactions = {}
J = 0.0

for v in form.vertices():
    if not form.vertex_attribute(v, "is_support"):
        continue

    px, py, pz = form.vertex_coordinates(v)
    P = Rhino.Geometry.Point3d(px, py, pz)

    Rx = Ry = Rz = 0.0

    for nbr in form.vertex_neighbors(v):
        q = form.edge_attribute((v, nbr), "q")
        if q is None:
            q = form.edge_attribute((nbr, v), "q")
        if q is None:
            q = 0.0

        nx, ny, nz = form.vertex_coordinates(nbr)
        Q = Rhino.Geometry.Point3d(nx, ny, nz)

        d = vec(P, Q)
        Lij = length(d)
        u = unit(d)

        # axial force magnitude proxy
        N = q * Lij

        # contribution to node equilibrium
        Rx += N * u[0]
        Ry += N * u[1]
        Rz += N * u[2]

    # Reaction is opposite of internal edge resultants
    Rx, Ry, Rz = -Rx, -Ry, -Rz

    reactions[v] = (Rx, Ry, Rz)
    J += math.sqrt(Rx*Rx + Ry*Ry)

print("Objective J (sum horizontal reactions):", J)


# -----------------------------
# 5) Draw in Rhino
# -----------------------------
# draw form edges
lines = []
for u, v in form.edges():
    ax, ay, az = form.vertex_coordinates(u)
    bx, by, bz = form.vertex_coordinates(v)
    A = Rhino.Geometry.Point3d(ax, ay, az)
    B = Rhino.Geometry.Point3d(bx, by, bz)
    lines.append((A, B))

draw_lines(lines, DRAW_LAYER, Color.White)

# draw reaction arrows at supports (scaled)
arrows = []
arrow_scale = 0.15  # tweak
for v, (Rx, Ry, Rz) in reactions.items():
    x, y, z = form.vertex_coordinates(v)
    A = Rhino.Geometry.Point3d(x, y, z)
    B = Rhino.Geometry.Point3d(x + Rx * arrow_scale, y + Ry * arrow_scale, z + Rz * arrow_scale)
    arrows.append((A, B))

draw_arrows(arrows, REACTION_LAYER, Color.Cyan)

sc.doc.Views.Redraw()
print("Done.")
