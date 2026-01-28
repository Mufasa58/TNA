# -*- coding: utf-8 -*-
# Rhino 8 + Python 3.9
# Pure COMPAS-TNA funicular test (no RhinoVAULT)

import Rhino
import scriptcontext as sc
import System
import System.Drawing as SD

from compas_tna.diagrams import FormDiagram, ForceDiagram
from compas_tna.equilibrium import vertical_from_zmax


# -----------------------------
# PARAMETERS
# -----------------------------
nx, ny = 12, 12          # Number of cells
dx, dy = 7.0, 7.0        # Cell size (total span = nx*dx, ny*dy)
zmax = 3.0               # Maximum sag/rise of funicular
thickness = 0.3          # Slab thickness for envelope

# Support configuration: 
# 'x_sides' = support at x=min and x=max (barrel vault spanning X)
# 'y_sides' = support at y=min and y=max (barrel vault spanning Y)
# 'all_sides' = support all four edges
# Or use a list like ['x_min', 'x_max', 'y_min', 'y_max'] for custom
support_config = 'x_sides'

# Non-uniform loads (optional): dict of vertex_key -> load_multiplier
# Leave empty {} for uniform loading
custom_loads = {}  # e.g., {50: 2.0, 51: 2.0} to double load at vertices 50, 51


# -----------------------------
# 1) Build a plan topology
# -----------------------------
form = FormDiagram.from_meshgrid(dx=dx, dy=dy, nx=nx, ny=ny)

# -----------------------------
# 2) Supports (boundary)
# -----------------------------
# Two parallel sides: x=0 and x=max (like a barrel vault spanning in X)
# Note: from_meshgrid creates a grid from (0,0) to (dx, dy), NOT (0,0) to (dx*nx, dy*ny)

# First, find the actual bounding box of the form diagram
all_x = [form.vertex_coordinates(v)[0] for v in form.vertices()]
all_y = [form.vertex_coordinates(v)[1] for v in form.vertices()]
min_x, max_x = min(all_x), max(all_x)
min_y, max_y = min(all_y), max(all_y)

print(f"Form diagram bounds: X=[{min_x:.2f}, {max_x:.2f}], Y=[{min_y:.2f}, {max_y:.2f}]")

for v in form.vertices():
    x, y, z = form.vertex_coordinates(v)
    
    # Check each boundary condition
    on_x_min = abs(x - min_x) < 1e-9
    on_x_max = abs(x - max_x) < 1e-9
    on_y_min = abs(y - min_y) < 1e-9
    on_y_max = abs(y - max_y) < 1e-9
    
    if support_config == 'x_sides':
        # Support at x=min and x=max (barrel vault spanning in X direction)
        is_support = on_x_min or on_x_max
    elif support_config == 'y_sides':
        # Support at y=min and y=max (barrel vault spanning in Y direction)
        is_support = on_y_min or on_y_max
    elif support_config == 'all_sides':
        # All four sides supported
        is_support = on_x_min or on_x_max or on_y_min or on_y_max
    elif isinstance(support_config, list):
        # Custom: list of sides, e.g., ['x_min', 'x_max', 'y_min']
        is_support = False
        if 'x_min' in support_config: is_support = is_support or on_x_min
        if 'x_max' in support_config: is_support = is_support or on_x_max
        if 'y_min' in support_config: is_support = is_support or on_y_min
        if 'y_max' in support_config: is_support = is_support or on_y_max
    else:
        is_support = False
    
    form.vertex_attribute(v, "is_support", is_support)

# Use actual dimensions for later reference
total_x = max_x - min_x
total_y = max_y - min_y

# -----------------------------
# 3) Apply loads (uniform by default)
# -----------------------------
# Default load is 1.0 per vertex area; modify with custom_loads
for v in form.vertices():
    load = 1.0
    if v in custom_loads:
        load = custom_loads[v]
    form.vertex_attribute(v, "pz", load)

# -----------------------------
# 4) Vertical equilibrium (funicular)
# -----------------------------
# vertical_from_zmax handles the equilibrium computation internally
form_eq, scale = vertical_from_zmax(form, zmax=zmax, kmax=200, xtol=0.001, rtol=1e-6, density=1.0, display=False)

# -----------------------------
# 5) Create Force Diagram (after equilibrium)
# -----------------------------
# Create force diagram from the equilibrated form
force = ForceDiagram.from_formdiagram(form_eq)

# -----------------------------
# 6) Compute envelope (intrados/extrados)
# -----------------------------
# Store envelope z-values as vertex attributes
half_t = thickness / 2.0
for v in form_eq.vertices():
    x, y, z = form_eq.vertex_coordinates(v)
    form_eq.vertex_attribute(v, "z_intrados", z - half_t)
    form_eq.vertex_attribute(v, "z_extrados", z + half_t)

# -----------------------------
# RHINO DRAWING HELPERS
# -----------------------------
doc = sc.doc

def ensure_layer(name, color):
    """Create layer if it doesn't exist, return layer index."""
    idx = doc.Layers.FindByFullPath(name, True)
    if idx < 0:
        idx = doc.Layers.Add(name, color)
    return idx

def add_curve_to_layer(pt0, pt1, layer_idx):
    """Add a line/curve between two points on specified layer."""
    line = Rhino.Geometry.Line(pt0, pt1)
    obj_id = doc.Objects.AddLine(line)
    if obj_id != System.Guid.Empty:
        obj = doc.Objects.FindId(obj_id)
        if obj:
            obj.Attributes.LayerIndex = layer_idx
            obj.CommitChanges()

def add_point_to_layer(pt, layer_idx):
    """Add a point on specified layer."""
    obj_id = doc.Objects.AddPoint(pt)
    if obj_id != System.Guid.Empty:
        obj = doc.Objects.FindId(obj_id)
        if obj:
            obj.Attributes.LayerIndex = layer_idx
            obj.CommitChanges()

def add_mesh_to_layer(vertices, faces, layer_idx):
    """Create a Rhino mesh from vertices and faces."""
    mesh = Rhino.Geometry.Mesh()
    for v in vertices:
        mesh.Vertices.Add(v[0], v[1], v[2])
    for f in faces:
        if len(f) == 3:
            mesh.Faces.AddFace(f[0], f[1], f[2])
        elif len(f) == 4:
            mesh.Faces.AddFace(f[0], f[1], f[2], f[3])
    mesh.Normals.ComputeNormals()
    mesh.Compact()
    obj_id = doc.Objects.AddMesh(mesh)
    if obj_id != System.Guid.Empty:
        obj = doc.Objects.FindId(obj_id)
        if obj:
            obj.Attributes.LayerIndex = layer_idx
            obj.CommitChanges()

# -----------------------------
# 7) Draw Form Diagram (thrust network)
# -----------------------------
layer_form = ensure_layer("TNA::Form", SD.Color.Blue)
layer_supports = ensure_layer("TNA::Supports", SD.Color.Red)

for u, v in form_eq.edges():
    x1, y1, z1 = form_eq.vertex_coordinates(u)
    x2, y2, z2 = form_eq.vertex_coordinates(v)
    p1 = Rhino.Geometry.Point3d(x1, y1, z1)
    p2 = Rhino.Geometry.Point3d(x2, y2, z2)
    add_curve_to_layer(p1, p2, layer_form)

# Draw support points
for v in form_eq.vertices():
    if form_eq.vertex_attribute(v, "is_support"):
        x, y, z = form_eq.vertex_coordinates(v)
        add_point_to_layer(Rhino.Geometry.Point3d(x, y, z), layer_supports)

# -----------------------------
# 8) Draw Force Diagram (offset to the side)
# -----------------------------
layer_force = ensure_layer("TNA::Force", SD.Color.Green)

# Offset force diagram to the right of the form
force_offset_x = total_x + 20.0
force_offset_y = 0.0

for u, v in force.edges():
    x1, y1, z1 = force.vertex_coordinates(u)
    x2, y2, z2 = force.vertex_coordinates(v)
    p1 = Rhino.Geometry.Point3d(x1 + force_offset_x, y1 + force_offset_y, 0)
    p2 = Rhino.Geometry.Point3d(x2 + force_offset_x, y2 + force_offset_y, 0)
    add_curve_to_layer(p1, p2, layer_force)

# -----------------------------
# 9) Draw Envelope (intrados & extrados meshes)
# -----------------------------
layer_intrados = ensure_layer("TNA::Intrados", SD.Color.Orange)
layer_extrados = ensure_layer("TNA::Extrados", SD.Color.Purple)
layer_thrust = ensure_layer("TNA::ThrustMesh", SD.Color.Cyan)

# Build vertex index mapping
vertex_keys = list(form_eq.vertices())
key_to_index = {key: i for i, key in enumerate(vertex_keys)}

# Get faces from form diagram
faces = []
for fkey in form_eq.faces():
    face_verts = form_eq.face_vertices(fkey)
    if face_verts:
        faces.append([key_to_index[v] for v in face_verts])

# Thrust surface (middle)
verts_thrust = []
for v in vertex_keys:
    x, y, z = form_eq.vertex_coordinates(v)
    verts_thrust.append((x, y, z))
add_mesh_to_layer(verts_thrust, faces, layer_thrust)

# Intrados (bottom of slab)
verts_intrados = []
for v in vertex_keys:
    x, y, _ = form_eq.vertex_coordinates(v)
    z_int = form_eq.vertex_attribute(v, "z_intrados")
    verts_intrados.append((x, y, z_int))
add_mesh_to_layer(verts_intrados, faces, layer_intrados)

# Extrados (top of slab)
verts_extrados = []
for v in vertex_keys:
    x, y, _ = form_eq.vertex_coordinates(v)
    z_ext = form_eq.vertex_attribute(v, "z_extrados")
    verts_extrados.append((x, y, z_ext))
add_mesh_to_layer(verts_extrados, faces, layer_extrados)

# -----------------------------
# 10) Refresh and report
# -----------------------------
doc.Views.Redraw()

print("=" * 50)
print("TNA Analysis Complete")
print("=" * 50)
print(f"Grid: {nx} x {ny} cells")
print(f"Span: {total_x:.1f} x {total_y:.1f} units")
print(f"Support config: {support_config}")
print(f"zmax: {zmax} | scale: {scale:.4f}")
print(f"Slab thickness: {thickness}")
print(f"Vertices: {form_eq.number_of_vertices()} | Edges: {form_eq.number_of_edges()}")
print("=" * 50)
print("Layers created:")
print("  - TNA::Form (blue) - Thrust network edges")
print("  - TNA::Supports (red) - Support points")
print("  - TNA::Force (green) - Force diagram")
print("  - TNA::ThrustMesh (cyan) - Middle surface mesh")
print("  - TNA::Intrados (orange) - Bottom of slab")
print("  - TNA::Extrados (purple) - Top of slab")
print("=" * 50)

