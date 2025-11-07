from __future__ import annotations

import csv
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import List

from .config import BenchConfig, SceneConfig, SolverConfig


@dataclass
class BenchResult:
    scene: str
    solver: str
    contacts: int
    total_ms: float
    warm_ms: float
    iteration_ms: float
    assembly_ms: float
    iterations: int
    residual: float


def run_single(
    cfg: BenchConfig,
    scene: SceneConfig,
    solver: SolverConfig,
    run_index: int,
    temp_dir: Path,
) -> BenchResult:
    temp_dir.mkdir(parents=True, exist_ok=True)
    csv_path = temp_dir / f"{scene.name}_{solver.name}_{run_index}.csv"
    args: List[str] = [
        str(cfg.simple_bench),
        f"--scene={scene.name}",
        f"--csv={csv_path}",
        f"--solver={solver.name}",
    ]
    args.extend(scene.args)
    args.extend(solver.solver_args)

    try:
        subprocess.run(args, check=True, capture_output=True, text=True)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(
            f"simple_bench failed for {scene.name}/{solver.name}:\n"
            f"stdout:\n{exc.stdout}\n"
            f"stderr:\n{exc.stderr}"
        ) from exc

    with csv_path.open() as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise RuntimeError(f"No rows produced for {scene.name}/{solver.name}")
    row = rows[-1]
    return BenchResult(
        scene=row["scene"],
        solver=row["solver"],
        contacts=int(row.get("contacts", 0)),
        total_ms=float(row.get("total_ms", 0.0)),
        warm_ms=float(row.get("warm_ms", 0.0)),
        iteration_ms=float(row.get("iteration_ms", 0.0)),
        assembly_ms=float(row.get("assembly_ms", 0.0)),
        iterations=int(row.get("iterations", 0)),
        residual=float(row.get("residual", 0.0)),
    )


def run_bench(cfg: BenchConfig) -> List[BenchResult]:
    results: List[BenchResult] = []
    temp_dir = cfg.output.csv_path.parent / "tmp"
    for run in range(cfg.runs):
        for scene in cfg.scenes:
            for solver in cfg.solvers:
                results.append(run_single(cfg, scene, solver, run, temp_dir))
    return results
