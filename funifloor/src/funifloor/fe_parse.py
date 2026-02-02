"""
Parse CalculiX output files (.dat, .frd) to extract stress resultants.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
import numpy as np


@dataclass
class ElementResults:
    """Results for a single element."""
    elem_id: int
    # Membrane forces (force per unit length)
    Nx: float = 0.0
    Ny: float = 0.0
    Nxy: float = 0.0
    # Bending moments (moment per unit length)
    Mx: float = 0.0
    My: float = 0.0
    Mxy: float = 0.0
    # Transverse shear (force per unit length)
    Vx: float = 0.0
    Vy: float = 0.0
    # Integration point stresses (top/bottom surfaces)
    stress_top: Optional[np.ndarray] = None
    stress_bot: Optional[np.ndarray] = None


def parse_dat_file(dat_path: Path) -> Dict[str, Any]:
    """
    Parse CalculiX .dat file for stress results.
    
    The .dat file contains tabular output requested via *EL PRINT.
    For shell elements, stresses are given at integration points on top/bottom surfaces.
    
    Returns:
        Dict with element_results, displacements, etc.
    """
    if not dat_path.exists():
        raise FileNotFoundError(f"DAT file not found: {dat_path}")

    with open(dat_path, "r") as f:
        content = f.read()

    results = {
        "element_results": {},
        "node_displacements": {},
        "raw_sections": {},
    }

    # Parse element stresses section
    # Look for stress output blocks
    stress_pattern = re.compile(
        r"stresses.*?element\s+int\.pt\.\s+.*?\n(.*?)(?=\n\s*\n|\Z)",
        re.DOTALL | re.IGNORECASE
    )

    matches = stress_pattern.findall(content)
    for match in matches:
        _parse_stress_block(match, results["element_results"])

    # Parse section forces if present (SOF output)
    sof_pattern = re.compile(
        r"section forces.*?\n(.*?)(?=\n\s*\n|\Z)",
        re.DOTALL | re.IGNORECASE
    )
    sof_matches = sof_pattern.findall(content)
    for match in sof_matches:
        _parse_section_forces(match, results["element_results"])

    # Parse node displacements
    disp_pattern = re.compile(
        r"displacements.*?node\s+.*?\n(.*?)(?=\n\s*\n|\Z)",
        re.DOTALL | re.IGNORECASE
    )
    disp_matches = disp_pattern.findall(content)
    for match in disp_matches:
        _parse_displacements(match, results["node_displacements"])

    return results


def _parse_stress_block(block: str, element_results: Dict[int, ElementResults]):
    """Parse a stress output block."""
    lines = block.strip().split("\n")
    for line in lines:
        parts = line.split()
        if len(parts) < 6:
            continue
        try:
            elem_id = int(parts[0])
            # int_pt = int(parts[1])
            # Stresses: Sxx, Syy, Szz, Sxy, Sxz, Syz
            sxx = float(parts[2]) if len(parts) > 2 else 0.0
            syy = float(parts[3]) if len(parts) > 3 else 0.0
            # szz = float(parts[4]) if len(parts) > 4 else 0.0
            sxy = float(parts[5]) if len(parts) > 5 else 0.0

            if elem_id not in element_results:
                element_results[elem_id] = ElementResults(elem_id=elem_id)

            # Accumulate stresses (will average later)
            er = element_results[elem_id]
            # For shells, we approximate membrane forces from mid-surface stress
            # This is simplified; proper integration needed for accurate values
            er.Nx += sxx
            er.Ny += syy
            er.Nxy += sxy

        except (ValueError, IndexError):
            continue


def _parse_section_forces(block: str, element_results: Dict[int, ElementResults]):
    """
    Parse section forces output (if available).
    Section forces give direct Nx, Ny, Nxy, Mx, My, Mxy, Vx, Vy.
    """
    lines = block.strip().split("\n")
    for line in lines:
        parts = line.split()
        if len(parts) < 8:
            continue
        try:
            elem_id = int(parts[0])

            if elem_id not in element_results:
                element_results[elem_id] = ElementResults(elem_id=elem_id)

            er = element_results[elem_id]
            # Section forces format varies; this is a common layout
            er.Nx = float(parts[1]) if len(parts) > 1 else 0.0
            er.Ny = float(parts[2]) if len(parts) > 2 else 0.0
            er.Nxy = float(parts[3]) if len(parts) > 3 else 0.0
            er.Mx = float(parts[4]) if len(parts) > 4 else 0.0
            er.My = float(parts[5]) if len(parts) > 5 else 0.0
            er.Mxy = float(parts[6]) if len(parts) > 6 else 0.0
            er.Vx = float(parts[7]) if len(parts) > 7 else 0.0
            er.Vy = float(parts[8]) if len(parts) > 8 else 0.0

        except (ValueError, IndexError):
            continue


def _parse_displacements(block: str, node_displacements: Dict[int, np.ndarray]):
    """Parse displacement output block."""
    lines = block.strip().split("\n")
    for line in lines:
        parts = line.split()
        if len(parts) < 4:
            continue
        try:
            node_id = int(parts[0])
            ux = float(parts[1])
            uy = float(parts[2])
            uz = float(parts[3])
            node_displacements[node_id] = np.array([ux, uy, uz])
        except (ValueError, IndexError):
            continue


def compute_moments_from_stresses(
    element_results: Dict[int, ElementResults],
    thickness: float,
) -> None:
    """
    Compute bending moments from top/bottom surface stresses.
    
    For a shell with stresses at +t/2 (top) and -t/2 (bottom):
    Nx = t * (σ_top + σ_bot) / 2  (membrane)
    Mx = t² / 6 * (σ_top - σ_bot)  (bending)
    
    This modifies element_results in place.
    """
    # This is a placeholder; actual implementation depends on
    # how stresses are extracted from integration points
    pass


def results_to_arrays(
    element_results: Dict[int, ElementResults],
    n_elements: int,
) -> Dict[str, np.ndarray]:
    """
    Convert element results dict to numpy arrays.
    
    Returns:
        Dict with arrays: Nx, Ny, Nxy, Mx, My, Mxy, Vx, Vy
    """
    Nx = np.zeros(n_elements)
    Ny = np.zeros(n_elements)
    Nxy = np.zeros(n_elements)
    Mx = np.zeros(n_elements)
    My = np.zeros(n_elements)
    Mxy = np.zeros(n_elements)
    Vx = np.zeros(n_elements)
    Vy = np.zeros(n_elements)

    for elem_id, er in element_results.items():
        idx = elem_id - 1  # CalculiX is 1-indexed
        if 0 <= idx < n_elements:
            Nx[idx] = er.Nx
            Ny[idx] = er.Ny
            Nxy[idx] = er.Nxy
            Mx[idx] = er.Mx
            My[idx] = er.My
            Mxy[idx] = er.Mxy
            Vx[idx] = er.Vx
            Vy[idx] = er.Vy

    return {
        "Nx": Nx,
        "Ny": Ny,
        "Nxy": Nxy,
        "Mx": Mx,
        "My": My,
        "Mxy": Mxy,
        "Vx": Vx,
        "Vy": Vy,
    }
