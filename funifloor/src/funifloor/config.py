"""
Configuration loading and validation.
"""

import json
from pathlib import Path
from typing import Any, Dict


def load_config(path: str | Path) -> Dict[str, Any]:
    """Load and validate config from JSON file."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    with open(path, "r") as f:
        config = json.load(f)

    # Validate required sections
    required = ["geometry", "structure", "loads", "segmentation", "output"]
    for section in required:
        if section not in config:
            raise ValueError(f"Config missing required section: {section}")

    return config


def config_to_pipeline_args(config: Dict[str, Any]) -> Dict[str, Any]:
    """Convert config dict to pipeline function arguments."""
    geo = config["geometry"]
    struct = config["structure"]
    loads = config["loads"]
    seg = config["segmentation"]
    solver = config.get("solver", {})
    output = config["output"]

    # Compute total load
    g = 9.81
    density = struct["material"]["density"]
    thickness = struct["thickness"]
    self_weight = density * thickness * g / 1000.0 if loads.get("self_weight", True) else 0.0
    additional = loads.get("additional_dead", 0.0)
    live = loads.get("live", 0.0)
    factor = loads.get("load_factor", 1.0)
    load_per_area = factor * (self_weight + additional + live)

    return {
        "span_x": geo["span_x"],
        "span_y": geo["span_y"],
        "n_bays_x": geo["n_bays_x"],
        "n_bays_y": geo["n_bays_y"],
        "rise": geo.get("rise_target", 0.5),
        "thickness": thickness,
        "E": struct["material"]["E"],
        "nu": struct["material"]["nu"],
        "density": density,
        "load_per_area": load_per_area,
        "n_panels_x": seg["n_panels_x"],
        "n_panels_y": seg["n_panels_y"],
        "max_panel_weight": seg.get("max_panel_weight"),
        "max_panel_dimension": seg.get("max_panel_dimension"),
        "output_dir": output["dir"],
        "output_formats": output.get("formats", ["obj", "vtk", "json", "csv"]),
        "use_calculix": solver.get("use_calculix", True),
        "fallback_if_missing": solver.get("fallback_if_missing", True),
        "ccx_path": solver.get("ccx_path"),
    }
