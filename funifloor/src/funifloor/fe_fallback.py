"""
Fallback bending analysis when CalculiX is not available.
Computes curvature-based "bending risk indicator" — NOT real bending moments.
"""

from __future__ import annotations

from typing import Dict, List, Any
import numpy as np


def compute_bending_risk_indicator(
    vertices: np.ndarray,
    faces: List[List[int]],
    edge_forces: Dict[tuple, float],
    thickness: float,
) -> Dict[str, np.ndarray]:
    """
    Compute a proxy bending risk indicator based on:
    1. Surface curvature (Gaussian + mean)
    2. Membrane force gradients
    
    ⚠️ WARNING: This is NOT a real FE bending analysis!
    It's a rough indicator for preliminary design only.
    
    Args:
        vertices: (N, 3) vertex coordinates
        faces: List of face vertex index lists
        edge_forces: Dict mapping (u, v) to edge force magnitude
        thickness: Shell thickness
    
    Returns:
        Dict with per-element risk indicators:
        - bending_risk: combined indicator
        - curvature_indicator: from surface curvature
        - gradient_indicator: from force gradients
        - is_real_bending: False (clearly marked)
    """
    n_faces = len(faces)

    # Compute face centroids and normals
    centroids = np.zeros((n_faces, 3))
    normals = np.zeros((n_faces, 3))

    for i, face in enumerate(faces):
        pts = vertices[face]
        centroids[i] = pts.mean(axis=0)

        if len(face) >= 3:
            a = pts[1] - pts[0]
            b = pts[2] - pts[0]
            n = np.cross(a, b)
            norm = np.linalg.norm(n)
            if norm > 1e-12:
                normals[i] = n / norm
            else:
                normals[i] = [0, 0, 1]

    # Build face adjacency
    edge_to_faces: Dict[tuple, List[int]] = {}
    for fi, face in enumerate(faces):
        for j in range(len(face)):
            v0, v1 = face[j], face[(j + 1) % len(face)]
            edge = tuple(sorted([v0, v1]))
            if edge not in edge_to_faces:
                edge_to_faces[edge] = []
            edge_to_faces[edge].append(fi)

    # Curvature indicator: normal variation between adjacent faces
    curvature_indicator = np.zeros(n_faces)
    for edge, face_list in edge_to_faces.items():
        if len(face_list) == 2:
            f0, f1 = face_list
            # Angle between normals
            dot = np.clip(np.dot(normals[f0], normals[f1]), -1, 1)
            angle = np.arccos(dot)
            curvature_indicator[f0] += angle
            curvature_indicator[f1] += angle

    # Normalize by face valence
    face_valence = np.zeros(n_faces)
    for face_list in edge_to_faces.values():
        for fi in face_list:
            face_valence[fi] += 1
    face_valence = np.maximum(face_valence, 1)
    curvature_indicator /= face_valence

    # Force gradient indicator: variation of edge forces around each face
    gradient_indicator = np.zeros(n_faces)
    for fi, face in enumerate(faces):
        forces_around = []
        for j in range(len(face)):
            v0, v1 = face[j], face[(j + 1) % len(face)]
            f = edge_forces.get((v0, v1), edge_forces.get((v1, v0), 0.0))
            forces_around.append(f)

        if len(forces_around) > 0:
            forces_arr = np.array(forces_around)
            # Coefficient of variation as gradient proxy
            if forces_arr.mean() > 1e-12:
                gradient_indicator[fi] = forces_arr.std() / forces_arr.mean()

    # Combined bending risk (weighted sum, normalized)
    # Higher curvature + higher force gradient = higher bending risk
    curv_norm = curvature_indicator / (curvature_indicator.max() + 1e-12)
    grad_norm = gradient_indicator / (gradient_indicator.max() + 1e-12)

    bending_risk = 0.6 * curv_norm + 0.4 * grad_norm

    # Scale by thickness inverse (thinner = higher bending stress for same curvature)
    bending_risk *= (0.1 / thickness) if thickness > 0 else 1.0

    print(f"[FALLBACK] Computed bending risk indicator (NOT real FE)")
    print(f"[FALLBACK] Max curvature indicator: {curvature_indicator.max():.4f}")
    print(f"[FALLBACK] Max gradient indicator: {gradient_indicator.max():.4f}")
    print(f"[FALLBACK] Max bending risk: {bending_risk.max():.4f}")

    return {
        "bending_risk": bending_risk,
        "curvature_indicator": curvature_indicator,
        "gradient_indicator": gradient_indicator,
        "is_real_bending": False,
        "warning": "This is a proxy indicator, NOT real bending moment analysis",
    }


def estimate_pseudo_moments(
    bending_risk: np.ndarray,
    membrane_forces: Dict[str, np.ndarray],
    thickness: float,
) -> Dict[str, np.ndarray]:
    """
    Generate pseudo-moments proportional to bending risk.
    These are NOT physically meaningful — only for visualization scaling.
    
    Real moments would require proper shell FE analysis.
    """
    n = len(bending_risk)

    # Scale pseudo-moments by bending risk and average membrane force
    Nx = membrane_forces.get("Nx", np.zeros(n))
    Ny = membrane_forces.get("Ny", np.zeros(n))
    avg_membrane = (np.abs(Nx) + np.abs(Ny)) / 2

    # Pseudo-moment ~ risk * membrane * thickness
    # This is dimensionally nonsense but provides relative scaling
    scale = thickness * 0.1

    Mx_pseudo = bending_risk * avg_membrane * scale
    My_pseudo = bending_risk * avg_membrane * scale
    Mxy_pseudo = bending_risk * avg_membrane * scale * 0.5

    return {
        "Mx": Mx_pseudo,
        "My": My_pseudo,
        "Mxy": Mxy_pseudo,
        "is_pseudo": True,
        "warning": "PSEUDO-MOMENTS: Not from FE analysis",
    }
