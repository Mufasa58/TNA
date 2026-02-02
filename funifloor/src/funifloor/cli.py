"""
Command-line interface for funifloor.
"""

import argparse
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(
        prog="funifloor",
        description="Funicular Prefabricated Floor: TNA + Segmentation + FE Verification"
    )
    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # demo command
    demo_parser = subparsers.add_parser("demo", help="Run a demo with default parameters")
    demo_parser.add_argument("--output", "-o", type=str, default="./out",
                             help="Output directory")

    # run command
    run_parser = subparsers.add_parser("run", help="Run with config file")
    run_parser.add_argument("--config", "-c", type=str, required=True,
                            help="Path to config JSON file")
    run_parser.add_argument("--output", "-o", type=str, default=None,
                            help="Override output directory")

    # check-solver command
    check_parser = subparsers.add_parser("check-solver", help="Check if CalculiX is available")

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(0)

    if args.command == "demo":
        run_demo(args.output)
    elif args.command == "run":
        run_config(args.config, args.output)
    elif args.command == "check-solver":
        check_solver()
    else:
        parser.print_help()
        sys.exit(1)


def run_demo(output_dir: str):
    """Run demo with default parameters."""
    from funifloor.pipeline import run_pipeline

    print("=" * 60)
    print("FUNIFLOOR DEMO")
    print("=" * 60)

    results = run_pipeline(
        span_x=5.0,
        span_y=5.0,
        n_bays_x=5,
        n_bays_y=5,
        rise=0.5,
        thickness=0.08,
        E=30000.0,
        nu=0.2,
        density=2500.0,
        load_per_area=5.0,
        n_panels_x=3,
        n_panels_y=3,
        output_dir=output_dir,
        use_calculix=True,
        fallback_if_missing=True,
    )

    print("\n" + "=" * 60)
    print("DEMO COMPLETE")
    print(f"Outputs written to: {results['output_dir']}")
    print("=" * 60)


def run_config(config_path: str, output_override: str | None):
    """Run with config file."""
    from funifloor.config import load_config
    from funifloor.pipeline import run_pipeline_from_config

    config = load_config(config_path)
    if output_override:
        config["output"]["dir"] = output_override

    print("=" * 60)
    print("FUNIFLOOR RUN")
    print(f"Config: {config_path}")
    print("=" * 60)

    results = run_pipeline_from_config(config)

    print("\n" + "=" * 60)
    print("RUN COMPLETE")
    print(f"Outputs written to: {results['output_dir']}")
    print("=" * 60)


def check_solver():
    """Check if CalculiX is available."""
    from funifloor.fe_run import check_ccx_available

    available, version = check_ccx_available()

    if available:
        print(f"✓ CalculiX is available: {version}")
    else:
        print("✗ CalculiX (ccx) not found on PATH")
        print("\nInstall instructions:")
        print("  macOS:   brew install calculix-ccx")
        print("  Ubuntu:  sudo apt install calculix-ccx")
        print("  conda:   conda install -c conda-forge calculix")


if __name__ == "__main__":
    main()
