"""
Main pipeline: orchestrates TNA → Segmentation → FE → Export.
"""

from __future__ import annotations

from pathlib import Path
from datetime import datetime
from typing import Dict, Any, Optional
import numpy as np

from funifloor.tna import run_tna
from funifloor.geometry import form_to_arrays, extract_edge_forces
from funifloor.segmentation import run_segmentation
from funifloor.fe_export import write_inp, estimate_pressure
from funifloor.fe_run import check_ccx_available, run_ccx
from funifloor.fe_parse import parse_dat_file, results_to_arrays
from funifloor.fe_fallback import compute_bending_risk_indicator, estimate_pseudo_moments
from funifloor.export import (
    write_obj,
    write_vtk,
    write_json_topology,
    write_panels_json,
    write_results_csv,
    write_summary,
)
from funifloor.config import config_to_pipeline_args


def run_pipeline(
    span_x: float = 5.0,
    span_y: float = 5.0,
    n_bays_x: int = 5,
    n_bays_y: int = 5,
    rise: float = 0.5,
    thickness: float = 0.08,
    E: float = 30000.0,
    nu: float = 0.2,
    density: float = 2500.0,
    load_per_area: float = 5.0,
    n_panels_x: int = 3,
    n_panels_y: int = 3,
    max_panel_weight: Optional[float] = None,
    max_panel_dimension: Optional[float] = None,
    output_dir: str = "./out",
    output_formats: list | None = None,
    use_calculix: bool = True,
    fallback_if_missing: bool = True,
    ccx_path: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Run the complete funifloor pipeline.
    
    Args:
        span_x, span_y: Floor dimensions (m)
        n_bays_x, n_bays_y: TNA grid subdivisions
        rise: Target max height (m)
        thickness: Shell thickness (m)
        E: Young's modulus (MPa)
        nu: Poisson's ratio
        density: Material density (kg/m³)
        load_per_area: Total load (kN/m²)
        n_panels_x, n_panels_y: Segmentation grid
        max_panel_weight: Transport limit (kg)
        max_panel_dimension: Transport limit (m)
        output_dir: Base output directory
        output_formats: List of formats to export
        use_calculix: Whether to try CalculiX for FE
        fallback_if_missing: Use fallback if ccx unavailable
        ccx_path: Explicit path to ccx executable
    
    Returns:
        Dict with results and output paths
    """
    if output_formats is None:
        output_formats = ["obj", "vtk", "json", "csv"]

    # Create output directory with timestamp
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = Path(output_dir) / run_id
    out_path.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print(f"FUNIFLOOR PIPELINE — Run ID: {run_id}")
    print("=" * 60)

    results = {
        "run_id": run_id,
        "output_dir": str(out_path),
    }

    # =========================================================================
    # PHASE 1: TNA Form-Finding
    # =========================================================================
    print("\n[PHASE 1] TNA Form-Finding")
    print("-" * 40)

    tna_result = run_tna(
        span_x=span_x,
        span_y=span_y,
        n_bays_x=n_bays_x,
        n_bays_y=n_bays_y,
        rise=rise,
        load_per_area=load_per_area,
        support_type="corners",
    )

    form = tna_result["form"]
    supports = tna_result["supports"]
    results["tna"] = {
        "n_vertices": form.number_of_vertices(),
        "n_edges": form.number_of_edges(),
        "n_faces": form.number_of_faces(),
        "n_supports": len(supports),
        "scale": tna_result["scale"],
        "z_range": tna_result["z_range"],
    }

    # Convert to arrays
    arrays = form_to_arrays(form)
    vertices = arrays["vertices"]
    faces = arrays["faces"]
    edges = arrays["edges"]
    key_to_idx = arrays["key_to_idx"]

    # Map supports to indices
    support_indices = [key_to_idx[s] for s in supports]

    # Extract edge forces
    edge_forces = extract_edge_forces(form)

    # =========================================================================
    # PHASE 2: Segmentation
    # =========================================================================
    print("\n[PHASE 2] Segmentation")
    print("-" * 40)

    seg_result = run_segmentation(
        vertices=vertices,
        faces=faces,
        n_panels_x=n_panels_x,
        n_panels_y=n_panels_y,
        thickness=thickness,
        density=density,
        max_panel_weight=max_panel_weight,
        max_panel_dimension=max_panel_dimension,
    )

    panels = seg_result["panels"]
    results["segmentation"] = {
        "n_panels": seg_result["n_panels"],
        "total_area": seg_result["total_area"],
        "total_weight": seg_result["total_weight"],
        "weight_range": seg_result["weight_range"],
        "n_violations": len(seg_result["violations"]),
    }

    # =========================================================================
    # PHASE 3: FE Verification
    # =========================================================================
    print("\n[PHASE 3] FE Verification")
    print("-" * 40)

    fe_results = None
    fe_method = "none"

    if use_calculix:
        ccx_available, ccx_msg = check_ccx_available(ccx_path)
        print(f"[FE] CalculiX check: {ccx_msg}")

        if ccx_available:
            # Write .inp file
            inp_path = out_path / "funifloor.inp"
            pressure = estimate_pressure(load_per_area)

            write_inp(
                inp_path=inp_path,
                vertices=vertices,
                faces=faces,
                supports=support_indices,
                thickness=thickness,
                E=E,
                nu=nu,
                pressure=pressure,
            )

            # Run CalculiX
            success, msg, dat_path = run_ccx(inp_path, ccx_path)

            if success and dat_path:
                print(f"[FE] Parsing results from {dat_path}")
                try:
                    parsed = parse_dat_file(dat_path)
                    fe_results = results_to_arrays(
                        parsed["element_results"],
                        len(faces),
                    )
                    fe_method = "calculix"
                    print(f"[FE] Parsed {len(parsed['element_results'])} element results")
                except Exception as e:
                    print(f"[FE] Failed to parse results: {e}")
            else:
                print(f"[FE] CalculiX failed: {msg}")

    # Fallback if FE failed or unavailable
    if fe_results is None and fallback_if_missing:
        print("[FE] Using fallback bending risk indicator")
        fallback = compute_bending_risk_indicator(
            vertices=vertices,
            faces=faces,
            edge_forces=edge_forces,
            thickness=thickness,
        )

        # Create pseudo membrane forces from edge forces
        n_faces_count = len(faces)
        Nx = np.zeros(n_faces_count)
        Ny = np.zeros(n_faces_count)
        for fi, face in enumerate(faces):
            forces = []
            for j in range(len(face)):
                v0, v1 = face[j], face[(j + 1) % len(face)]
                f = edge_forces.get((v0, v1), edge_forces.get((v1, v0), 0.0))
                forces.append(f)
            avg_f = np.mean(forces) if forces else 0.0
            Nx[fi] = avg_f
            Ny[fi] = avg_f

        pseudo_moments = estimate_pseudo_moments(
            fallback["bending_risk"],
            {"Nx": Nx, "Ny": Ny},
            thickness,
        )

        fe_results = {
            "Nx": Nx,
            "Ny": Ny,
            "Nxy": np.zeros(n_faces_count),
            "Mx": pseudo_moments["Mx"],
            "My": pseudo_moments["My"],
            "Mxy": pseudo_moments["Mxy"],
            "Vx": np.zeros(n_faces_count),
            "Vy": np.zeros(n_faces_count),
            "bending_risk": fallback["bending_risk"],
        }
        fe_method = "fallback"

    results["fe"] = {
        "method": fe_method,
        "n_elements": len(faces),
    }

    if fe_results:
        results["fe"]["Mx_max"] = float(np.abs(fe_results.get("Mx", [0])).max())
        results["fe"]["My_max"] = float(np.abs(fe_results.get("My", [0])).max())
        results["fe"]["Nx_max"] = float(np.abs(fe_results.get("Nx", [0])).max())

    # =========================================================================
    # PHASE 4: Export
    # =========================================================================
    print("\n[PHASE 4] Export")
    print("-" * 40)

    if "obj" in output_formats:
        write_obj(out_path / "mesh.obj", vertices, faces)

    if "vtk" in output_formats and fe_results:
        # Prepare cell data for VTK
        cell_data = {}
        for key in ["Nx", "Ny", "Nxy", "Mx", "My", "Mxy", "Vx", "Vy", "bending_risk"]:
            if key in fe_results:
                arr = fe_results[key]
                # meshio expects list of arrays per cell block
                n_quads = sum(1 for f in faces if len(f) == 4)
                n_tris = sum(1 for f in faces if len(f) == 3)
                if n_quads > 0 and n_tris > 0:
                    cell_data[key] = [arr[:n_quads], arr[n_quads:n_quads + n_tris]]
                elif n_quads > 0:
                    cell_data[key] = [arr]
                elif n_tris > 0:
                    cell_data[key] = [arr]

        write_vtk(out_path / "mesh.vtk", vertices, faces, cell_data=cell_data)

    if "json" in output_formats:
        write_json_topology(
            out_path / "topology.json",
            vertices, faces, edges, support_indices, edge_forces,
        )
        write_panels_json(out_path / "panels.json", panels)

    if "csv" in output_formats and fe_results:
        write_results_csv(out_path / "results.csv", len(faces), fe_results)

    # Write summary
    write_summary(out_path / "summary.txt", results)

    print("\n" + "=" * 60)
    print("PIPELINE COMPLETE")
    print(f"Outputs: {out_path}")
    print("=" * 60)

    return results


def run_pipeline_from_config(config: Dict[str, Any]) -> Dict[str, Any]:
    """Run pipeline from config dict."""
    args = config_to_pipeline_args(config)
    return run_pipeline(**args)
