"""
Run CalculiX solver and manage external process.
"""

from __future__ import annotations

import subprocess
import shutil
from pathlib import Path
from typing import Tuple, Optional


def check_ccx_available(ccx_path: str | None = None) -> Tuple[bool, str]:
    """
    Check if CalculiX (ccx) is available on the system.
    
    Args:
        ccx_path: Optional explicit path to ccx executable
    
    Returns:
        (is_available, version_string or error message)
    """
    ccx = ccx_path or shutil.which("ccx")

    if ccx is None:
        return False, "ccx not found on PATH"

    try:
        result = subprocess.run(
            [ccx, "-v"],
            capture_output=True,
            text=True,
            timeout=10,
        )
        # ccx -v prints version to stderr typically
        version = result.stderr.strip() or result.stdout.strip()
        if "CalculiX" in version or "ccx" in version.lower():
            return True, version.split("\n")[0]
        return True, f"ccx found at {ccx}"
    except FileNotFoundError:
        return False, f"ccx not found at {ccx}"
    except subprocess.TimeoutExpired:
        return False, "ccx check timed out"
    except Exception as e:
        return False, str(e)


def run_ccx(
    inp_path: Path,
    ccx_path: str | None = None,
    timeout: int = 300,
) -> Tuple[bool, str, Optional[Path]]:
    """
    Run CalculiX on an input file.
    
    Args:
        inp_path: Path to .inp file
        ccx_path: Optional explicit path to ccx executable
        timeout: Timeout in seconds
    
    Returns:
        (success, message, dat_path or None)
    """
    ccx = ccx_path or shutil.which("ccx")

    if ccx is None:
        return False, "ccx not found", None

    inp_path = Path(inp_path)
    if not inp_path.exists():
        return False, f"Input file not found: {inp_path}", None

    # ccx expects job name without extension
    job_name = inp_path.stem
    work_dir = inp_path.parent

    print(f"[FE] Running CalculiX: {ccx} {job_name}")

    try:
        result = subprocess.run(
            [ccx, job_name],
            cwd=work_dir,
            capture_output=True,
            text=True,
            timeout=timeout,
        )

        # Check for output files
        dat_path = work_dir / f"{job_name}.dat"
        frd_path = work_dir / f"{job_name}.frd"

        if result.returncode != 0:
            error_msg = result.stderr or result.stdout
            return False, f"ccx failed: {error_msg[:500]}", None

        if dat_path.exists():
            print(f"[FE] CalculiX completed successfully")
            return True, "Success", dat_path
        else:
            return False, "ccx ran but no output files generated", None

    except subprocess.TimeoutExpired:
        return False, f"ccx timed out after {timeout}s", None
    except Exception as e:
        return False, str(e), None
