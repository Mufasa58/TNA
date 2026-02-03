# fan_vault_bake.py
# RunnerV2-compatible: exposes run() with no args.

import os
import System
import Rhino
import scriptcontext as sc


# ==========================
# USER SETTINGS (EDIT THESE)
# ==========================

OUT_DIR = r"/Users/mmg/dev/tna_playground/JSON Files"

THRUST_JSON   = os.path.join(OUT_DIR, "RhinoVAULT_THRUST_SOLVED.json")
INTRADOS_JSON = os.path.join(OUT_DIR, "RhinoVAULT_INTRADOS.json")
EXTRADOS_JSON = os.path.join(OUT_DIR, "RhinoVAULT_EXTRADOS.json")

BAKE_AS_BREP = True          # try Brep first
BAKE_BOTH = True   # bake BOTH brep-attempt and raw mesh for comparison

BAKE_EDGES   = True          # bake thrust edges as curves
CLEAR_LAYER  = False         # if True, deletes existing objs on the target layers before baking


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
        if gid != Rhino.Geometry.Guid.Empty:
            ids.append(str(gid))
    return ids


# ==========================
# ENTRYPOINT FOR RunnerV2
# ==========================
def run():
    from compas.data import json_load

    dbg = {}
    try:
        for p in (THRUST_JSON, INTRADOS_JSON, EXTRADOS_JSON):
            if not os.path.isfile(p):
                raise FileNotFoundError("Missing JSON: {}".format(p))

        thrust   = json_load(THRUST_JSON)
        intrados = json_load(INTRADOS_JSON)
        extrados = json_load(EXTRADOS_JSON)

        dbg["loaded"] = {
            "thrust": THRUST_JSON,
            "intrados": INTRADOS_JSON,
            "extrados": EXTRADOS_JSON,
        }

        # layers
        L_THRUST   = "TNA::THRUST"
        L_INTRADOS = "TNA::INTRADOS"
        L_EXTRADOS = "TNA::EXTRADOS"
        L_EDGES    = "TNA::THRUST_EDGES"

        if CLEAR_LAYER:
            dbg["cleared"] = {
                "thrust": clear_objects_on_layer(L_THRUST),
                "intrados": clear_objects_on_layer(L_INTRADOS),
                "extrados": clear_objects_on_layer(L_EXTRADOS),
                "edges": clear_objects_on_layer(L_EDGES),
            }

        # convert to Rhino meshes
        r_thrust   = compas_mesh_to_rhino_mesh(thrust)
        r_intrados = compas_mesh_to_rhino_mesh(intrados)
        r_extrados = compas_mesh_to_rhino_mesh(extrados)

        # bake
        baked = {}
        baked["thrust"]   = bake_mesh_and_or_brep(r_thrust,   L_THRUST,   "THRUST_SOLVED")
        baked["intrados"] = bake_mesh_and_or_brep(r_intrados, L_INTRADOS, "INTRADOS")
        baked["extrados"] = bake_mesh_and_or_brep(r_extrados, L_EXTRADOS, "EXTRADOS")

        if BAKE_EDGES:
            baked["edges"] = {
                "layer": ensure_layer(L_EDGES),
                "ids": bake_thrust_edges(thrust, L_EDGES, "thrust_edge"),
            }

        sc.doc.Views.Redraw()

        return {"ok": True, "baked": baked, "dbg": dbg}

    except Exception as e:
        import traceback
        return {"ok": False, "error": traceback.format_exc(), "dbg": dbg}
