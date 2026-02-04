# fan_vault_bake.py
# RunnerV2-compatible: exposes run() with no args.

import os
import math
import System
import Rhino
import Rhino.Geometry as rg
import scriptcontext as sc
import System.Drawing as Drawing

# ==========================
# USER SETTINGS (EDIT THESE)
# ==========================

OUT_DIR = r"/Users/mmg/dev/tna_playground/JSON Files"

THRUST_JSON   = os.path.join(OUT_DIR, "Pattern_simple_v2_THRUST_SOLVED.json")
INTRADOS_JSON = os.path.join(OUT_DIR, "Pattern_simple_v2_INTRADOS.json")
EXTRADOS_JSON = os.path.join(OUT_DIR, "Pattern_simple_v2_EXTRADOS.json")
FORCE_JSON = os.path.join(OUT_DIR, "Pattern_simple_v2_FORCE.json")


BAKE_AS_BREP = True          # try Brep first
BAKE_BOTH = True   # bake BOTH brep-attempt and raw mesh for comparison

BAKE_EDGES   = True          # bake thrust edges as curves
BAKE_FORCE   = True          # bake force diagram edges as curves
CLEAR_LAYER  = False         # if True, deletes existing objs on the target layers before baking

BAKE_REACTIONS = True

# Scale for arrow length in Rhino units (pure visualization).
REACTION_SCALE = 0.35

# Arrow head size in Rhino units (visual)
ARROW_SIZE = 0.12


# ==========================
# HELPERS
# ==========================

def ensure_layer(fullname):
    """
    Ensure a (possibly nested) Rhino layer exists, return its full name.
    Example fullname: "TNA::THRUST"
    """
    parts = fullname.split("::")
    parent_id = System.Guid.Empty
    current_name = ""
    for i, p in enumerate(parts):
        current_name = p if i == 0 else current_name + "::" + p
        layer_index = sc.doc.Layers.FindByFullPath(current_name, True)
        if layer_index >= 0:
            parent_id = sc.doc.Layers[layer_index].Id
            continue

        layer = Rhino.DocObjects.Layer()
        layer.Name = p
        if i > 0:
            layer.ParentLayerId = parent_id
        layer_index = sc.doc.Layers.Add(layer)
        parent_id = sc.doc.Layers[layer_index].Id

    return current_name


def clear_objects_on_layer(layer_fullname):
    idx = sc.doc.Layers.FindByFullPath(layer_fullname, True)
    if idx < 0:
        return 0
    layer = sc.doc.Layers[idx]
    to_delete = []
    for obj in sc.doc.Objects:
        if obj.Attributes.LayerIndex == layer.LayerIndex:
            to_delete.append(obj.Id)
    for gid in to_delete:
        sc.doc.Objects.Delete(gid, True)
    return len(to_delete)


def compas_mesh_to_rhino_mesh(mesh):
    """
    Convert a COMPAS Mesh-like object to Rhino.Geometry.Mesh.
    Works for compas.datastructures.Mesh and diagrams (they inherit Mesh).
    """
    rm = Rhino.Geometry.Mesh()
    vmap = {}

    # add vertices
    for k in mesh.vertices():
        x, y, z = mesh.vertex_coordinates(k)
        vmap[k] = rm.Vertices.Add(x, y, z)

    # add faces
    for f in mesh.faces():
        vs = mesh.face_vertices(f)
        if not vs:
            continue
        if len(vs) == 3:
            rm.Faces.AddFace(vmap[vs[0]], vmap[vs[1]], vmap[vs[2]])
        elif len(vs) == 4:
            rm.Faces.AddFace(vmap[vs[0]], vmap[vs[1]], vmap[vs[2]], vmap[vs[3]])
        else:
            # ngon: triangulate fan from first vertex
            a = vs[0]
            for i in range(1, len(vs) - 1):
                rm.Faces.AddFace(vmap[a], vmap[vs[i]], vmap[vs[i + 1]])

    rm.Normals.ComputeNormals()
    rm.Compact()
    return rm


def bake_mesh_and_or_brep(rmesh, layer_fullname, name):
    """
    If BAKE_BOTH:
      - always bake the raw mesh (stable reference)
      - also try Brep-from-mesh and bake it if it succeeds
    Else:
      - behave like before (try brep then fallback to mesh)
    """
    layer_fullname = ensure_layer(layer_fullname)
    layer_index = sc.doc.Layers.FindByFullPath(layer_fullname, True)

    attr = Rhino.DocObjects.ObjectAttributes()
    attr.LayerIndex = layer_index
    attr.Name = name

    baked = {"layer": layer_fullname, "name": name, "mesh_id": None, "brep_id": None, "brep_ok": False}

    # Always compute some stability improvements for the mesh
    rmesh.Normals.ComputeNormals()
    rmesh.Compact()

    # Always bake mesh if BAKE_BOTH, or if brep attempt fails
    def bake_mesh():
        gid = sc.doc.Objects.AddMesh(rmesh, attr)
        if gid != System.Guid.Empty:
            baked["mesh_id"] = str(gid)

    if BAKE_BOTH:
        # bake mesh reference first
        bake_mesh()

        # try brep as a comparison object
        if BAKE_AS_BREP:
            brep = Rhino.Geometry.Brep.CreateFromMesh(rmesh, True)
            if brep:
                gid = sc.doc.Objects.AddBrep(brep, attr)
                if gid != System.Guid.Empty:
                    baked["brep_ok"] = True
                    baked["brep_id"] = str(gid)

        return baked

    # old behavior: try brep first, else mesh
    if BAKE_AS_BREP:
        brep = Rhino.Geometry.Brep.CreateFromMesh(rmesh, True)
        if brep:
            gid = sc.doc.Objects.AddBrep(brep, attr)
            if gid != System.Guid.Empty:
                baked["brep_ok"] = True
                baked["brep_id"] = str(gid)
                return baked

    # fallback to mesh
    bake_mesh()
    return baked



def bake_thrust_edges(mesh_like, layer_fullname, name_prefix="edge"):
    """
    Bake edges as line curves (polyline segments).
    """
    layer_fullname = ensure_layer(layer_fullname)
    layer_index = sc.doc.Layers.FindByFullPath(layer_fullname, True)

    attr = Rhino.DocObjects.ObjectAttributes()
    attr.LayerIndex = layer_index
    attr.Name = name_prefix

    ids = []
    for u, v in mesh_like.edges():
        a = Rhino.Geometry.Point3d(*mesh_like.vertex_coordinates(u))
        b = Rhino.Geometry.Point3d(*mesh_like.vertex_coordinates(v))
        crv = Rhino.Geometry.LineCurve(a, b)
        gid = sc.doc.Objects.AddCurve(crv, attr)
        if gid != System.Guid.Empty:
            ids.append(str(gid))
    return ids

def clamp01(x):
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)

def lerp(a, b, t):
    return a + (b - a) * t

def color_ramp_blue_to_red(t):
    # t: 0..1  -> blue-ish to red-ish
    t = clamp01(t)
    r = int(lerp(30, 230, t))
    g = int(lerp(80,  40, t))
    b = int(lerp(230, 30, t))
    return Drawing.Color.FromArgb(r, g, b)

def bake_edges_styled(mesh_like, layer_name, name_prefix, style_fn):
    """
    style_fn(u, v, i): returns (System.Drawing.Color, plot_weight_float)
    """
    doc = Rhino.RhinoDoc.ActiveDoc
    if doc is None:
        raise Exception("No active Rhino document.")

    ids = []
    idx = 0

    for u, v in mesh_like.edges():
        a = mesh_like.vertex_coordinates(u)
        b = mesh_like.vertex_coordinates(v)

        # for line geometry
        line = rg.Line(rg.Point3d(a[0], a[1], a[2]), rg.Point3d(b[0], b[1], b[2]))
        crv = line.ToNurbsCurve()

        color, w = style_fn(u, v, idx)

        att = Rhino.DocObjects.ObjectAttributes()
        layer_fullname = ensure_layer(layer_name)
        layer_index = sc.doc.Layers.FindByFullPath(layer_fullname, True)
        att.LayerIndex = layer_index
        att.Name = "{}_{}".format(name_prefix, idx)

        # Object color
        att.ColorSource = Rhino.DocObjects.ObjectColorSource.ColorFromObject
        att.ObjectColor = color

        # Plot weight (useful for print/PDF; viewport thickness depends on display mode)
        if w is not None:
            att.PlotWeightSource = Rhino.DocObjects.ObjectPlotWeightSource.PlotWeightFromObject
            att.PlotWeight = float(w)

        gid = doc.Objects.AddCurve(crv, att)
        if gid != System.Guid.Empty:
            ids.append(gid)

        idx += 1

    doc.Views.Redraw()
    return ids


def bake_reaction_arrows(thrust, layer_fullname, scale=1.0, arrow_size=0.1, name_prefix="R"):
    """
    Bake reaction vectors at fixed (support) vertices.
    Uses vertex attributes: is_fixed, Rx, Ry, Rz.
    Draws a line + 2 small 'wings' as an arrow head.
    """
    layer_fullname = ensure_layer(layer_fullname)
    layer_index = sc.doc.Layers.FindByFullPath(layer_fullname, True)

    ids = []
    for v in thrust.vertices():
        if not thrust.vertex_attribute(v, "is_fixed"):
            continue

        Rx = thrust.vertex_attribute(v, "Rx")
        Ry = thrust.vertex_attribute(v, "Ry")
        Rz = thrust.vertex_attribute(v, "Rz")

        # Skip if reactions not present
        if Rx is None or Ry is None or Rz is None:
            continue

        x, y, z = thrust.vertex_coordinates(v)
        p0 = Rhino.Geometry.Point3d(x, y, z)

        # reaction direction (scaled for visualization)
        dx, dy, dz = float(Rx) * scale, float(Ry) * scale, float(Rz) * scale
        p1 = Rhino.Geometry.Point3d(x + dx, y + dy, z + dz)

        # main shaft
        shaft = Rhino.Geometry.LineCurve(p0, p1)

        attr = Rhino.DocObjects.ObjectAttributes()
        attr.LayerIndex = layer_index
        attr.Name = "{}_{}".format(name_prefix, v)
        # (optional) color supports distinctly
        attr.ColorSource = Rhino.DocObjects.ObjectColorSource.ColorFromObject
        attr.ObjectColor = Drawing.Color.FromArgb(0, 0, 0)  # black

        gid = sc.doc.Objects.AddCurve(shaft, attr)
        if gid != System.Guid.Empty:
            ids.append(str(gid))

        # --- arrow head (two wings) ---
        # Build two small lines near p1, roughly perpendicular to the vector
        vec = Rhino.Geometry.Vector3d(dx, dy, dz)
        if vec.Length > 1e-9:
            vec.Unitize()

            # pick an arbitrary "up" to build a perpendicular basis
            up = Rhino.Geometry.Vector3d(0, 0, 1)
            if abs(Rhino.Geometry.Vector3d.Multiply(vec, up)) > 0.95:
                up = Rhino.Geometry.Vector3d(0, 1, 0)

            side = Rhino.Geometry.Vector3d.CrossProduct(vec, up)
            side.Unitize()

            back = Rhino.Geometry.Vector3d(-vec.X, -vec.Y, -vec.Z)

            w = float(arrow_size)
            p_w1 = p1 + back * (2.0 * w) + side * (1.0 * w)
            p_w2 = p1 + back * (2.0 * w) - side * (1.0 * w)

            wing1 = Rhino.Geometry.LineCurve(p1, p_w1)
            wing2 = Rhino.Geometry.LineCurve(p1, p_w2)

            gid1 = sc.doc.Objects.AddCurve(wing1, attr)
            gid2 = sc.doc.Objects.AddCurve(wing2, attr)
            if gid1 != System.Guid.Empty: ids.append(str(gid1))
            if gid2 != System.Guid.Empty: ids.append(str(gid2))

    return ids


# ==========================
# ENTRYPOINT FOR RunnerV2
# ==========================
def run():
    from compas.data import json_load

    dbg = {}
    try:
        for p in (THRUST_JSON, INTRADOS_JSON, EXTRADOS_JSON, FORCE_JSON):
            if not os.path.isfile(p):
                raise FileNotFoundError("Missing JSON: {}".format(p))

        thrust   = json_load(THRUST_JSON)
        intrados = json_load(INTRADOS_JSON)
        extrados = json_load(EXTRADOS_JSON)
        force = json_load(FORCE_JSON)

        dbg["loaded"] = {
            "thrust": THRUST_JSON,
            "intrados": INTRADOS_JSON,
            "extrados": EXTRADOS_JSON,
            "force": FORCE_JSON,
        }

        # layers
        L_THRUST   = "TNA::THRUST"
        L_INTRADOS = "TNA::INTRADOS"
        L_EXTRADOS = "TNA::EXTRADOS"
        L_EDGES    = "TNA::THRUST_EDGES"
        L_FORCE    = "TNA::FORCE_EDGES"
        L_REACT = "TNA::REACTIONS"

        baked = {}
        if BAKE_REACTIONS:
            baked["reactions"] = {
                "layer": ensure_layer(L_REACT),
                "ids": bake_reaction_arrows(thrust, L_REACT, scale=REACTION_SCALE, arrow_size=ARROW_SIZE, name_prefix="R"),
                "scale": REACTION_SCALE,
                "arrow_size": ARROW_SIZE,
            }

        if CLEAR_LAYER:
            dbg["cleared"] = {
                "thrust": clear_objects_on_layer(L_THRUST),
                "intrados": clear_objects_on_layer(L_INTRADOS),
                "extrados": clear_objects_on_layer(L_EXTRADOS),
                "edges": clear_objects_on_layer(L_EDGES),
                "force": clear_objects_on_layer(L_FORCE),
                "reactions": clear_objects_on_layer(L_REACT),
            }

        # convert to Rhino meshes
        r_thrust   = compas_mesh_to_rhino_mesh(thrust)
        r_intrados = compas_mesh_to_rhino_mesh(intrados)
        r_extrados = compas_mesh_to_rhino_mesh(extrados)
        r_force    = compas_mesh_to_rhino_mesh(force)
        
        dbg["rhino_meshes"] = {
            "thrust": r_thrust.Vertices.Count,
            "intrados": r_intrados.Vertices.Count,
            "extrados": r_extrados.Vertices.Count,
            "force": r_force.Vertices.Count,
        }

        # bake
        baked["thrust"]   = bake_mesh_and_or_brep(r_thrust,   L_THRUST,   "THRUST_SOLVED")
        baked["intrados"] = bake_mesh_and_or_brep(r_intrados, L_INTRADOS, "INTRADOS")
        baked["extrados"] = bake_mesh_and_or_brep(r_extrados, L_EXTRADOS, "EXTRADOS")

        # --- THRUST EDGES colored by compression magnitude N ---
        L_THRUST_N = "TNA::THRUST_EDGES_N"

        # 1) collect N magnitudes for normalization
        Ns = []
        for u, v in thrust.edges():
            N = thrust.edge_attribute((u, v), "N")
            if N is None:
                # fallback: compute N = q * L
                q = thrust.edge_attribute((u, v), "q") or 0.0
                a = thrust.vertex_coordinates(u)
                b = thrust.vertex_coordinates(v)
                dx = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
                L = (dx[0]**2 + dx[1]**2 + dx[2]**2) ** 0.5
                N = q * L
            #Ns.append(abs(float(N)))
            Ns.append(math.log10(1.0 + abs(float(N))))




        Nmin = min(Ns) if Ns else 0.0
        Nmax = max(Ns) if Ns else 1.0
        
        
        Ncap = 0.18 * Nmax   # tune: 0.15–0.40
        Nmax = max(Nmax, Ncap)
        print("N range:", Nmin, Nmax)

        
        den  = (Nmax - Nmin) if (Nmax - Nmin) > 1e-12 else 1.0

        def style_thrust_N(u, v, i):
            N = thrust.edge_attribute((u, v), "N")
            if N is None:
                q = thrust.edge_attribute((u, v), "q") or 0.0
                a = thrust.vertex_coordinates(u)
                b = thrust.vertex_coordinates(v)
                dx = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
                L = (dx[0]**2 + dx[1]**2 + dx[2]**2) ** 0.5
                N = q * L


            
          
            
            #N = abs(float(N))
            N = math.log10(1.0 + abs(float(N))) # log scale, use this only when N ranges over multiple orders of magnitude

            N = min(N, Ncap)
            t = (N - Nmin) / (Ncap - Nmin + 1e-12)

            #t = (N - Nmin) / den  # 0..1
            t = t ** 1.0 # higher - lower forces pop, lower - higher forces 
            #t = 0.5 - 0.5 * math.cos(math.pi * t) # s-curve try1





            #blue red map does not really show differences in low N range well
            #here is a new map:
            
            def color_ramp_multi(t):
                    t = clamp01(t)
                    #stops = [
                    #    (0.00, (0,   0,  80)),   # deep blue
                    #    (0.25, (0, 120, 255)),   # bright blue
                    #    (0.50, (0, 255, 180)),   # cyan/green
                    #    (0.75, (255, 230, 0)),   # yellow
                    #    (1.00, (255, 40,  0)),   # red
                    #]
                    stops = [
                        (0.00, (  0,   0,  60)),
                        (0.125,(  0,  60, 160)),
                        (0.25, (  0, 120, 255)),
                        (0.375,(  0, 200, 255)),
                        (0.50, (  0, 255, 180)),
                        (0.625,(120, 255,  80)),
                        (0.75, (255, 230,   0)),
                        (0.875,(255, 140,   0)),
                        (1.00, (255,  40,   0)),
                    ]

                    # find segment
                    for i in range(len(stops)-1):
                        t0, c0 = stops[i]
                        t1, c1 = stops[i+1]
                        if t <= t1:
                            u = (t - t0) / (t1 - t0 + 1e-12)
                            r = int(lerp(c0[0], c1[0], u))
                            g = int(lerp(c0[1], c1[1], u))
                            b = int(lerp(c0[2], c1[2], u))
                            return Drawing.Color.FromArgb(r, g, b)
                    return Drawing.Color.FromArgb(255, 40, 0)


            # SETTINGS !!!
            col = color_ramp_multi(t)
            #col = color_ramp_blue_to_red(t) # color ramp
            w = lerp(0.05, 2.00, t) # plot weight
            return col, w

        baked["thrust_edges_N"] = {
            "layer": ensure_layer(L_THRUST_N),
            "ids": bake_edges_styled(thrust, L_THRUST_N, "thrustN", style_thrust_N),
            "Nmin": float(Nmin),
            "Nmax": float(Nmax),
        }


        if BAKE_FORCE:
            baked["force"] = {
                "layer": ensure_layer(L_FORCE),
                "ids": bake_thrust_edges(force, L_FORCE, "force_edge"),
            }
        sc.doc.Views.Redraw()

        return {"ok": True, "baked": baked, "dbg": dbg}

    except Exception as e:
        import traceback
        return {"ok": False, "error": traceback.format_exc(), "dbg": dbg}
