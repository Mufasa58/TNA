"""
Panel segmentation for prefabricated floor.
Strategy S1: Grid-based cuts in plan, mapped to surface.
"""

from __future__ import annotations

from typing import Dict, List, Any
from dataclasses import dataclass, field
import numpy as np


@dataclass
class Panel:
    """A prefabricated panel."""
    id: str
    vertices: np.ndarray  # (N, 3) array of 3D vertices
    vertex_indices: List[int]  # indices into global vertex array
    face_indices: List[int]  # indices into global face array
    boundary_indices: List[int]  # ordered boundary vertex indices
    area: float = 0.0
    centroid: np.ndarray = field(default_factory=lambda: np.zeros(3))
    bbox_size: np.ndarray = field(default_factory=lambda: np.zeros(3))
    weight: float = 0.0  # handling weight in kg

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "vertices": self.vertices.tolist(),
            "vertex_indices": self.vertex_indices,
            "face_indices": self.face_indices,
            "boundary_indices": self.boundary_indices,
            "area": self.area,
            "centroid": self.centroid.tolist(),
            "bbox_size": self.bbox_size.tolist(),
            "weight": self.weight,
        }


def segment_grid_s1(
    vertices: np.ndarray,
    faces: List[List[int]],
    n_panels_x: int,
    n_panels_y: int,
    thickness: float = 0.08,
    density: float = 2500.0,
) -> List[Panel]:
    """
    Strategy S1: Cut mesh into grid of panels in plan (X-Y).
    
    Args:
        vertices: (N, 3) vertex coordinates
        faces: List of face vertex index lists
        n_panels_x: Number of panel divisions in X
        n_panels_y: Number of panel divisions in Y
        thickness: Panel thickness for weight calculation
        density: Material density (kg/m³)
    
    Returns:
        List of Panel objects
    """
    # Compute XY bounding box
    xmin, xmax = vertices[:, 0].min(), vertices[:, 0].max()
    ymin, ymax = vertices[:, 1].min(), vertices[:, 1].max()

    dx = (xmax - xmin) / n_panels_x
    dy = (ymax - ymin) / n_panels_y

    # Compute face centroids for assignment
    face_centroids = []
    for face in faces:
        pts = vertices[face]
        face_centroids.append(pts.mean(axis=0))
    face_centroids = np.array(face_centroids)

    # Assign faces to panels
    panels = []
    for i in range(n_panels_x):
        for j in range(n_panels_y):
            panel_id = f"P_{i}_{j}"

            # Panel bounds in XY
            x0 = xmin + i * dx
            x1 = xmin + (i + 1) * dx
            y0 = ymin + j * dy
            y1 = ymin + (j + 1) * dy

            # Find faces whose centroid falls in this panel
            in_panel = (
                (face_centroids[:, 0] >= x0) & (face_centroids[:, 0] < x1) &
                (face_centroids[:, 1] >= y0) & (face_centroids[:, 1] < y1)
            )
            face_indices = np.where(in_panel)[0].tolist()

            if len(face_indices) == 0:
                continue

            # Collect vertices from these faces
            vertex_set = set()
            for fi in face_indices:
                vertex_set.update(faces[fi])
            vertex_indices = sorted(vertex_set)

            panel_vertices = vertices[vertex_indices]

            # Compute panel properties
            area = 0.0
            for fi in face_indices:
                face = faces[fi]
                pts = vertices[face]
                if len(face) == 4:
                    # Quad area
                    a1 = pts[1] - pts[0]
                    b1 = pts[2] - pts[0]
                    a2 = pts[2] - pts[0]
                    b2 = pts[3] - pts[0]
                    area += 0.5 * (np.linalg.norm(np.cross(a1, b1)) + np.linalg.norm(np.cross(a2, b2)))
                elif len(face) == 3:
                    a = pts[1] - pts[0]
                    b = pts[2] - pts[0]
                    area += 0.5 * np.linalg.norm(np.cross(a, b))

            centroid = panel_vertices.mean(axis=0)
            bbox_min = panel_vertices.min(axis=0)
            bbox_max = panel_vertices.max(axis=0)
            bbox_size = bbox_max - bbox_min

            weight = area * thickness * density

            # Find boundary vertices (simplified: vertices on panel XY edges)
            boundary_indices = _find_boundary_vertices(
                panel_vertices, vertex_indices, x0, x1, y0, y1
            )

            panel = Panel(
                id=panel_id,
                vertices=panel_vertices,
                vertex_indices=vertex_indices,
                face_indices=face_indices,
                boundary_indices=boundary_indices,
                area=area,
                centroid=centroid,
                bbox_size=bbox_size,
                weight=weight,
            )
            panels.append(panel)

    return panels


def _find_boundary_vertices(
    panel_vertices: np.ndarray,
    vertex_indices: List[int],
    x0: float, x1: float,
    y0: float, y1: float,
    tol: float = 1e-6,
) -> List[int]:
    """Find vertices on the panel boundary."""
    boundary = []
    for i, (vi, pt) in enumerate(zip(vertex_indices, panel_vertices)):
        on_edge = (
            abs(pt[0] - x0) < tol or
            abs(pt[0] - x1) < tol or
            abs(pt[1] - y0) < tol or
            abs(pt[1] - y1) < tol
        )
        if on_edge:
            boundary.append(vi)
    return boundary


def check_transport_limits(
    panels: List[Panel],
    max_weight: float | None = None,
    max_dimension: float | None = None,
) -> List[Dict[str, Any]]:
    """
    Check panels against transport/handling limits.
    
    Returns:
        List of violations (empty if all OK)
    """
    violations = []
    for panel in panels:
        if max_weight is not None and panel.weight > max_weight:
            violations.append({
                "panel_id": panel.id,
                "issue": "weight",
                "value": panel.weight,
                "limit": max_weight,
            })
        if max_dimension is not None:
            max_dim = panel.bbox_size.max()
            if max_dim > max_dimension:
                violations.append({
                    "panel_id": panel.id,
                    "issue": "dimension",
                    "value": max_dim,
                    "limit": max_dimension,
                })
    return violations


def run_segmentation(
    vertices: np.ndarray,
    faces: List[List[int]],
    n_panels_x: int,
    n_panels_y: int,
    thickness: float,
    density: float,
    max_panel_weight: float | None = None,
    max_panel_dimension: float | None = None,
) -> Dict[str, Any]:
    """
    Run segmentation and check limits.
    
    Returns:
        Dict with panels list, violations, summary
    """
    print(f"[SEG] Segmenting into {n_panels_x}x{n_panels_y} panels")

    panels = segment_grid_s1(
        vertices, faces, n_panels_x, n_panels_y, thickness, density
    )

    print(f"[SEG] Created {len(panels)} panels")

    violations = check_transport_limits(panels, max_panel_weight, max_panel_dimension)

    if violations:
        print(f"[SEG] WARNING: {len(violations)} transport limit violations")
        for v in violations[:3]:
            print(f"      {v['panel_id']}: {v['issue']} = {v['value']:.2f} > {v['limit']:.2f}")

    # Summary stats
    weights = [p.weight for p in panels]
    areas = [p.area for p in panels]

    return {
        "panels": panels,
        "violations": violations,
        "n_panels": len(panels),
        "total_area": sum(areas),
        "total_weight": sum(weights),
        "weight_range": (min(weights), max(weights)) if weights else (0, 0),
    }
