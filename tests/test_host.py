import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from model.rtl_check import wsl_path
from physics.catalog import load_bikes
from physics.cranking import Battery, CrankingModel, Cylinder, simulate

SIMULATION = ("cub110", 12.5, 0.028, 0.35, 60.0, 100.0, 1.5)


def wsl(command):
    finished = subprocess.run(["wsl", "-e", "bash", "-lc", f"cd '{wsl_path(ROOT / 'host')}' && {command}"], capture_output=True, text=True)
    assert finished.returncode == 0, finished.stdout + finished.stderr
    return finished.stdout


def test_host_unit_checks_under_sanitizers():
    assert "0 failed" in wsl("make check 2>&1 | tail -1")


def test_cpp_and_python_cranking_agree():
    bike_name, emf, resistance, leak, heat, start, seconds = SIMULATION
    lines = wsl(f"make host_test >/dev/null && ./host_test sim ../data {bike_name} {emf} {resistance} {leak} {heat} {start} {seconds}").split("\n")
    cpp = np.array([[float(x) for x in line.split()] for line in lines if line.strip()])
    model = CrankingModel(load_bikes()[bike_name], Battery(emf, resistance), Cylinder(leak * 1e-6, heat))
    record = simulate(model, np.radians(start), np.random.default_rng(0), jitter_deg=0.0, duration=seconds)
    python = np.array([[s.time, s.compression, s.density_ratio] for s in record.sparks])
    count = min(len(cpp), len(python))
    assert count >= 8
    assert np.max(np.abs(cpp[:count, 0] - python[:count, 0])) < 20e-6
    assert np.array_equal(cpp[:count, 1], python[:count, 1])
    assert np.max(np.abs(cpp[:count, 2] / python[:count, 2] - 1)) < 1e-4


@pytest.mark.slow
def test_case_grid_verdicts_match_truth():
    cases_file = ROOT / "web" / "data" / "cases.js"
    if not cases_file.exists():
        pytest.skip("run cases/build_cases.py first")
    from cases.score import score_cases
    summary = score_cases(cases_file)
    assert summary["wrong"] == 0