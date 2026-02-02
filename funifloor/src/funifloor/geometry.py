"""
Geometry utilities for mesh processing and conversion.
"""

from __future__ import annotations

from typing import Dict, List, Tuple, Any
import numpy as np

from compas_tna.diagrams import FormDiagram


def form_to_arrays(form: FormDiagram) -> Dict[str, np.ndarray]:
    """
    Convert FormDiagram to numpy arrays.
    
    Returns:
        Dict with:
        - vertices: (N, 3) array of vertex coordinates
        - faces: list of face vertex index lists
        - vertex_keys: list mapping array index to form vertex key
        - edges: (M, 2) array of edge vertex indices
    """
    vertex_keys = list(form.vertices())
    key_to_idx = {k: i for i, k in enumerate(vertex_keys)}

    vertices = np.array([form.vertex_coordinates(v) for v in vertex_keys])

    faces = []
    for fkey in form.faces():
        fv = form.face_vertices(fkey)
        faces.append([key_to_idx[v] for v in fv])

    edges = np.array([[key_to_idx[u], key_to_idx[v]] for u, v in form.edges()])

    return {
        "vertices": vertices,
        "faces": faces,
        "vertex_keys": vertex_keys,
        "key_to_idx": key_to_idx,
        "edges": edges,
    }


def compute_face_centroids(vertices: np.ndarray, faces: List[List[int]]) -> np.ndarray:
    """Compute centroid of each face."""
    centroids = []
    for face in faces:
        pts = vertices[face]
        centroids.append(pts.mean(axis=0))
    return np.array(centroids)


def compute_face_areas(vertices: np.ndarray, faces: List[List[int]]) -> np.ndarray:
    """Compute area of each face (quad or tri)."""
    areas = []
    for face in faces:
        pts = vertices[face]
        if len(face) == 3:
            # Triangle
            a = pts[1] - pts[0]
            b = pts[2] - pts[0]
            area = 0.5 * np.linalg.norm(np.cross(a, b))
        elif len(face) == 4:
            # Quad: split into two triangles
            a1 = pts[1] - pts[0]
            b1 = pts[2] - pts[0]
            a2 = pts[2] - pts[0]
            b2 = pts[3] - pts[0]
            area = 0.5 * (np.linalg.norm(np.cross(a1, b1)) + np.linalg.norm(np.cross(a2, b2)))
        else:
            # General polygon: sum triangles from centroid
            c = pts.mean(axis=0)
            area = 0.0
            for i in range(len(pts)):
                a = pts[i] - c
                b = pts[(i + 1) % len(pts)] - c
                area += 0.5 * np.linalg.norm(np.cross(a, b))
        areas.append(area)
    return np.array(areas)


def compute_face_normals(vertices: np.ndarray, faces: List[List[int]]) -> np.ndarray:
    """Compute unit normal of each face."""
    normals = []
    for face in faces:
        pts = vertices[face]
        if len(face) >= 3:
            a = pts[1] - pts[0]
            b = pts[2] - pts[0]
            n = np.cross(a, b)
            norm = np.linalg.norm(n)
            if norm > 1e-12:
                n = n / norm
            else:
                n = np.array([0.0, 0.0, 1.0])
        else:
            n = np.array([0.0, 0.0, 1.0])
        normals.append(n)
    return np.array(normals)


def compute_vertex_curvatures(
    vertices: np.ndarray,
    faces: List[List[int]],
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Estimate Gaussian and mean curvature at each vertex.
    Uses discrete curvature approximation.
    
    Returns:
        (gaussian_curvature, mean_curvature) arrays of shape (N,)
    """
    n_verts = len(vertices)
    angle_sum = np.zeros(n_verts)
    area_sum = np.zeros(n_verts)

    for face in faces:
        pts = vertices[face]
        n = len(face)
        face_area = 0.0

        # Compute face area
        if n >= 3:
            c = pts.mean(axis=0)
            for i in range(n):
                a = pts[i] - c
                b = pts[(i + 1) % n] - c
                face_area += 0.5 * np.linalg.norm(np.cross(a, b))

        # Accumulate angles at each vertex
        for i in range(n):
            v0 = face[(i - 1) % n]
            v1 = face[i]
            v2 = face[(i + 1) % n]

            a = vertices[v0] - vertices[v1]
            b = vertices[v2] - vertices[v1]
            cos_angle = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12)
            cos_angle = np.clip(cos_angle, -1, 1)
            angle = np.arccos(cos_angle)

            angle_sum[v1] += angle
            area_sum[v1] += face_area / n

    # Gaussian curvature: (2π - angle_sum) / area
    gaussian = (2 * np.pi - angle_sum) / (area_sum + 1e-12)

    # Mean curvature: simplified estimate from Laplacian
    # (This is a rough approximation)
    mean = np.zeros(n_verts)
    # Build adjacency
    adj = [[] for _ in range(n_verts)]
    for face in faces:
        for i, v in enumerate(face):
            v_next = face[(i + 1) % len(face)]
            if v_next not in adj[v]:
                adj[v].append(v_next)
            if v not in adj[v_next]:
                adj[v_next].append(v)

    for i in range(n_verts):
        if len(adj[i]) > 0:
            neighbors = vertices[adj[i]]
            laplacian = neighbors.mean(axis=0) - vertices[i]
            mean[i] = np.linalg.norm(laplacian)

    return gaussian, mean


def extract_edge_forces(form: FormDiagram) -> Dict[Tuple[int, int], float]:
    """
    Extract edge force magnitudes from FormDiagram.
    
    Returns:
        Dict mapping (u_idx, v_idx) to force magnitude
    """
    vertex_keys = list(form.vertices())
    key_to_idx = {k: i for i, k in enumerate(vertex_keys)}

    forces = {}
    for u, v in form.edges():
        # Try different attribute names
        f = None
        for attr in ("h", "f", "q"):
            val = form.edge_attribute((u, v), attr)
            if isinstance(val, (int, float)):
                f = abs(float(val))
                break
        if f is None:
            f = 0.0

        ui, vi = key_to_idx[u], key_to_idx[v]
        forces[(ui, vi)] = f
        forces[(vi, ui)] = f

    return forces
