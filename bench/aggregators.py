from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path
from statistics import median
from typing import Dict, Iterable, List

from .runner import BenchResult


def summarize(results: Iterable[BenchResult]) -> List[BenchResult]:
    grouped: Dict[tuple, List[BenchResult]] = defaultdict(list)
    for result in results:
        key = (result.scene, result.solver)
        grouped[key].append(result)

    summary: List[BenchResult] = []
    for (scene, solver), rows in grouped.items():
        summary.append(
            BenchResult(
                scene=scene,
                solver=solver,
                contacts=rows[0].contacts,
                total_ms=median(r.total_ms for r in rows),
                warm_ms=median(r.warm_ms for r in rows),
                iteration_ms=median(r.iteration_ms for r in rows),
                assembly_ms=median(r.assembly_ms for r in rows),
                iterations=int(median(r.iterations for r in rows)),
                residual=median(r.residual for r in rows),
                max_penetration=median(r.max_penetration for r in rows),
                max_joint_error=median(r.max_joint_error for r in rows),
                admc_drift=median(r.admc_drift for r in rows),
                tile_residual_min=median(r.tile_residual_min for r in rows),
                tile_residual_p95=median(r.tile_residual_p95 for r in rows),
                tile_residual_max=median(r.tile_residual_max for r in rows),
                tile_drift_min=median(r.tile_drift_min for r in rows),
                tile_drift_p95=median(r.tile_drift_p95 for r in rows),
                tile_drift_max=median(r.tile_drift_max for r in rows),
            )
        )
    return summary


def write_csv(path: Path, rows: List[BenchResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "scene",
                "contacts",
                "solver",
                "total_ms",
                "warm_ms",
                "iteration_ms",
                "assembly_ms",
                "iterations",
                "residual",
                "max_penetration",
                "max_joint_error",
                "admc_drift",
                "tile_residual_min",
                "tile_residual_p95",
                "tile_residual_max",
                "tile_drift_min",
                "tile_drift_p95",
                "tile_drift_max",
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "scene": row.scene,
                    "contacts": row.contacts,
                    "solver": row.solver,
                    "total_ms": f"{row.total_ms:.6f}",
                    "warm_ms": f"{row.warm_ms:.6f}",
                    "iteration_ms": f"{row.iteration_ms:.6f}",
                    "assembly_ms": f"{row.assembly_ms:.6f}",
                    "iterations": row.iterations,
                    "residual": f"{row.residual:.6f}",
                    "max_penetration": f"{row.max_penetration:.6f}",
                    "max_joint_error": f"{row.max_joint_error:.6f}",
                    "admc_drift": f"{row.admc_drift:.6f}",
                    "tile_residual_min": f"{row.tile_residual_min:.6f}",
                    "tile_residual_p95": f"{row.tile_residual_p95:.6f}",
                    "tile_residual_max": f"{row.tile_residual_max:.6f}",
                    "tile_drift_min": f"{row.tile_drift_min:.6f}",
                    "tile_drift_p95": f"{row.tile_drift_p95:.6f}",
                    "tile_drift_max": f"{row.tile_drift_max:.6f}",
                }
            )
