# Funicular Prefabricated Floor

A minimal but engineering-plausible pipeline for:
1. **Form-finding** a compression-dominated funicular floor (TNA)
2. **Segmentation** into prefabricated panels
3. **FE verification** with shell elements (bending moments + membrane forces)

## Installation

```bash
# Create virtual environment
python -m venv .venv
source .venv/bin/activate  # Linux/macOS
# .venv\Scripts\activate   # Windows

# Install package in editable mode
pip install -e .

# Optional: Install CalculiX for FE analysis
# macOS:   brew install calculix-ccx
# Ubuntu:  sudo apt install calculix-ccx
# conda:   conda install -c conda-forge calculix
```

## Quick Start

```bash
# Run demo (no external solver needed)
python -m funifloor demo

# Run with config file
python -m funifloor run --config config.example.json

# Check if CalculiX is available
python -m funifloor check-solver
```

## Usage as Library

```python
from funifloor.pipeline import run_pipeline

results = run_pipeline(
    span_x=5.0,
    span_y=5.0,
    n_bays_x=3,
    n_bays_y=3,
    rise=0.5,
    thickness=0.08,
    load_per_area=5.0,  # kN/m²
    n_panels_x=3,
    n_panels_y=3,
    output_dir="./out"
)
```

## Outputs

All outputs go to `./out/<run_id>/`:
- `mesh.obj` — thrust surface mesh
- `mesh.vtk` — mesh with stress resultants for ParaView
- `topology.json` — nodes, edges, forces
- `panels.json` — panel definitions + metadata
- `results.csv` — per-element stress resultants
- `summary.txt` — run summary

## FE Verification

### With CalculiX (recommended)
If `ccx` is available on PATH, the pipeline:
1. Exports shell mesh to `.inp` format
2. Runs CalculiX solver
3. Parses bending moments (Mx, My, Mxy) and membrane forces (Nx, Ny, Nxy)

### Fallback (no solver)
When ccx is unavailable, computes a **bending risk indicator** based on:
- Surface curvature (Gaussian + mean)
- Membrane force gradients
- **⚠️ This is NOT real bending analysis** — clearly labeled in outputs

## Known Limitations

1. **TNA assumptions**: Pure membrane equilibrium; no bending in form-finding
2. **Shell FE simplification**: Linear elastic, uniform thickness
3. **Segmentation S1**: Simple grid cuts; doesn't optimize for structural performance
4. **Joint modeling**: Panels assumed continuous (no joint stiffness reduction)
5. **Load model**: Uniform pressure only; no point loads or asymmetric cases
6. **Material**: Isotropic linear elastic; no cracking, creep, shrinkage

## Next Upgrades

- [ ] Segmentation S2: Cut along principal stress directions
- [ ] Panel joint modeling in FE (springs/gaps)
- [ ] Nonlinear material (concrete cracking)
- [ ] Transport/lifting check per panel
- [ ] Rhino/Grasshopper integration
- [ ] Parametric optimization (rise, thickness)
