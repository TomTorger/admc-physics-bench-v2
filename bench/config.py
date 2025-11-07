from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


@dataclass
class SolverConfig:
    name: str
    solver_args: List[str] = field(default_factory=list)


@dataclass
class SceneConfig:
    name: str
    args: List[str] = field(default_factory=list)


@dataclass
class OutputConfig:
    csv_path: Path


@dataclass
class BenchConfig:
    simple_bench: Path
    scenes: List[SceneConfig]
    solvers: List[SolverConfig]
    runs: int = 1
    output: OutputConfig = field(default_factory=lambda: OutputConfig(csv_path=Path("results.csv")))


def load_config(path: Path) -> BenchConfig:
    data = yaml.safe_load(path.read_text())
    simple_bench = Path(data["simple_bench"]).expanduser()

    scenes = [
        SceneConfig(name=scene["name"], args=scene.get("args", []))
        for scene in data.get("scenes", [])
    ]
    solvers = [
        SolverConfig(name=solver["name"], solver_args=solver.get("args", []))
        for solver in data.get("solvers", [])
    ]

    runs = int(data.get("runs", 1))
    csv_path = Path(data.get("output", {}).get("csv", "results/bench_summary.csv")).expanduser()

    return BenchConfig(
        simple_bench=simple_bench,
        scenes=scenes,
        solvers=solvers,
        runs=runs,
        output=OutputConfig(csv_path=csv_path),
    )
