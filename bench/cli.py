from __future__ import annotations

import argparse
from pathlib import Path

from .aggregators import summarize, write_csv
from .config import load_config
from .runner import run_bench


def main() -> int:
    parser = argparse.ArgumentParser(description="ADMC bench orchestrator")
    parser.add_argument("--config", required=True, type=Path, help="Path to bench YAML config")
    args = parser.parse_args()

    cfg = load_config(args.config)
    results = run_bench(cfg)
    summary = summarize(results)
    write_csv(cfg.output.csv_path, summary)
    print("\nScene           Solver        total(ms)  warm(ms)  iter(ms)  assembly  iter   residual")
    for row in summary:
        print(
            f"{row.scene:<15} {row.solver:<10} "
            f"{row.total_ms:>9.3f} {row.warm_ms:>9.3f} {row.iteration_ms:>9.3f} "
            f"{row.assembly_ms:>9.3f} "
            f"{row.iterations:>5} {row.residual:>10.4f}"
        )
    print(f"\nWrote {len(summary)} rows to {cfg.output.csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
