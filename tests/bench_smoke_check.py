import csv
import subprocess
import sys
from pathlib import Path

def main() -> int:
    if len(sys.argv) != 3:
        print("usage: bench_smoke_check.py <simple_bench> <csv_path>")
        return 1

    bench = Path(sys.argv[1])
    csv_path = Path(sys.argv[2])
    if csv_path.exists():
        csv_path.unlink()

    cmd = [
        str(bench),
        "--iterations=4",
        "--solver=both",
        "--scene=two_spheres",
        f"--csv={csv_path}",
    ]

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("simple_bench failed")
        print(result.stdout)
        print(result.stderr)
        return result.returncode

    if not csv_path.exists():
        print(f"missing csv: {csv_path}")
        return 1

    with csv_path.open() as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        print("CSV is empty")
        return 1

    required_cols = {
        "scene",
        "contacts",
        "solver",
        "total_ms",
        "warm_ms",
        "iteration_ms",
        "assembly_ms",
        "iterations",
        "residual",
    }
    missing = required_cols - set(rows[0].keys())
    if missing:
        print(f"missing columns: {missing}")
        return 1

    print(f"CSV ok: {len(rows)} rows")
    return 0

if __name__ == "__main__":
    sys.exit(main())
