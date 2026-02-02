"""
thrust_3.py — Pointer to funifloor package

The full funicular floor pipeline has been moved to a proper Python package:

    /Users/mmg/dev/tna_playground/funifloor/

To install and run:

    cd funifloor
    python -m venv .venv
    source .venv/bin/activate
    pip install -e .
    
    # Run demo
    python -m funifloor demo
    
    # Run with config
    python -m funifloor run --config config.example.json
    
    # Check if CalculiX is available
    python -m funifloor check-solver

See funifloor/README.md for full documentation.
"""

# For quick testing without installing, you can import directly:
# import sys
# sys.path.insert(0, "/Users/mmg/dev/tna_playground/funifloor/src")
# from funifloor.pipeline import run_pipeline
# results = run_pipeline()
