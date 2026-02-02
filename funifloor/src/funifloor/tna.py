"""
TNA form-finding using COMPAS TNA.
Generates a thrust network for a rectangular grid under uniform load.
"""

from __future__ import annotations

from typing import Dict, List, Tuple, Any
import numpy as np

from compas.datastructures import Mesh
from compas_tna.diagrams import FormDiagram, ForceDiagram
from compas_tna.equilibrium import horizontal_nodal, vertical_from_zmax


def create_grid_form(
    span_x: float,
    span_y: float,
    n_bays_x: int,
    n_bays_y: int,
) -> FormDiagram:
    """
    Create a rectangular grid FormDiagram.
    
    Args:
        span_x: Total span in X direction
        span_y: Total span in Y direction
        n_bays_x: Number of subdivisions in X
        n_bays_y: Number of subdivisions in Y
    
    Returns:
        FormDiagram with z=0 for all vertices
    """
    dx = span_x / n_bays_x
    dy = span_y / n_bays_y

    mesh = Mesh.from_meshgrid(dx=dx, nx=n_bays_x, dy=dy, ny=n_bays_y)
    form = FormDiagram.from_mesh(mesh)

    # Ensure z=0 for all vertices
    for v in form.vertices():
        x, y, _ = form.vertex_coordinates(v)
        form.vertex_attributes(v, ["x", "y", "z"], [x, y, 0.0])

    return form


def apply_corner_supports(form: FormDiagram) -> List[int]:
    """
    Fix the four corner vertices as supports.
    
    Returns:
        List of support vertex keys
    """
    # Find bounding box
    xs = [form.vertex_attribute(v, "x") for v in form.vertices()]
    ys = [form.vertex_attribute(v, "y") for v in form.vertices()]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)

    tol = 1e-6
    corners = [(xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax)]

    supports = []
    for v in form.vertices():
        x, y, _ = form.vertex_coordinates(v)
        for (cx, cy) in corners:
            if abs(x - cx) < tol and abs(y - cy) < tol:
                form.vertex_attribute(v, "is_fixed", True)
                supports.append(v)
                break

    return supports


def apply_edge_supports(form: FormDiagram) -> List[int]:
    """
    Fix all vertices on the boundary as supports.
    
    Returns:
        List of support vertex keys
    """
    supports = []
    for v in form.vertices():
        if form.vertex_attribute(v, "is_anchor") or form.is_vertex_on_boundary(v):
            form.vertex_attribute(v, "is_fixed", True)
            supports.append(v)
    return supports


def apply_uniform_load(
    form: FormDiagram,
    supports: List[int],
    load_per_area: float,
) -> None:
    """
    Apply uniform vertical load to all non-support vertices.
    Load is distributed as nodal loads based on tributary area.
    
    Args:
        form: FormDiagram to modify
        supports: List of support vertex keys (no load applied)
        load_per_area: Load per unit area (e.g., kN/m²)
    """
    sup_set = set(supports)

    for v in form.vertices():
        if v in sup_set:
            form.vertex_attributes(v, ["px", "py", "pz"], [0.0, 0.0, 0.0])
        else:
            # Estimate tributary area from neighboring faces
            area = 0.0
            for fkey in form.vertex_faces(v):
                if fkey is not None:
                    face_area = form.face_area(fkey)
                    n_verts = len(form.face_vertices(fkey))
                    area += face_area / n_verts

            pz = -load_per_area * area
            form.vertex_attributes(v, ["px", "py", "pz"], [0.0, 0.0, pz])


def _rebuild_edge_indices(diagram) -> None:
    """Ensure edge attribute '_i' is continuous 0..n-1."""
    for idx, edge in enumerate(diagram.edges()):
        diagram.edge_attribute(edge, "_i", idx)


def solve_equilibrium(
    form: FormDiagram,
    rise: float,
    alpha: float = 100.0,
    kmax: int = 200,
) -> Tuple[ForceDiagram, float]:
    """
    Solve horizontal and vertical equilibrium.
    
    Args:
        form: FormDiagram with loads and supports applied
        rise: Target maximum height (zmax)
        alpha: Horizontal equilibrium parameter
        kmax: Maximum iterations for horizontal solver
    
    Returns:
        Tuple of (ForceDiagram, scale factor)
    """
    # Rebuild edge indices for compatibility with compas_tna 0.7.x
    _rebuild_edge_indices(form)

    # Create force diagram
    force = ForceDiagram.from_formdiagram(form)
    _rebuild_edge_indices(force)

    # Solve horizontal equilibrium
    try:
        horizontal_nodal(form, force, alpha=alpha, kmax=kmax)
    except Exception as e:
        print(f"[TNA] horizontal_nodal failed: {e}")
        print("[TNA] Continuing with vertical equilibrium only...")
        # Initialize default force densities
        for edge in form.edges():
            if form.edge_attribute(edge, "q") is None:
                form.edge_attribute(edge, "q", 1.0)

    # Solve vertical equilibrium
    result = vertical_from_zmax(form, rise)
    if isinstance(result, tuple):
        _, scale = result
    else:
        scale = float(result)

    return force, scale


def run_tna(
    span_x: float,
    span_y: float,
    n_bays_x: int,
    n_bays_y: int,
    rise: float,
    load_per_area: float,
    support_type: str = "corners",
) -> Dict[str, Any]:
    """
    Run complete TNA form-finding.
    
    Args:
        span_x, span_y: Floor dimensions
        n_bays_x, n_bays_y: Grid subdivisions
        rise: Target max height
        load_per_area: Uniform load (kN/m²)
        support_type: "corners" or "edges"
    
    Returns:
        Dict with form, force, supports, scale
    """
    print(f"[TNA] Creating {n_bays_x}x{n_bays_y} grid, span {span_x}x{span_y}m")

    form = create_grid_form(span_x, span_y, n_bays_x, n_bays_y)

    if support_type == "edges":
        supports = apply_edge_supports(form)
    else:
        supports = apply_corner_supports(form)

    print(f"[TNA] {len(supports)} supports applied ({support_type})")

    apply_uniform_load(form, supports, load_per_area)
    print(f"[TNA] Load applied: {load_per_area} kN/m²")

    force, scale = solve_equilibrium(form, rise)
    print(f"[TNA] Equilibrium solved, scale={scale:.4f}")

    # Compute z statistics
    zs = [form.vertex_attribute(v, "z") for v in form.vertices()]
    zmin, zmax = min(zs), max(zs)
    print(f"[TNA] Z range: {zmin:.4f} to {zmax:.4f}")

    return {
        "form": form,
        "force": force,
        "supports": supports,
        "scale": scale,
        "z_range": (zmin, zmax),
    }
