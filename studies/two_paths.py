import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from cases.build_cases import BUILD, wsl_root
from cases.score import load

CASES_FILE = ROOT / "web" / "data" / "cases.js"
FRICTION_ERROR = 0.10
WORKERS = 6


def run(name, bike, flags):
    command = f"cd '{wsl_root()}' && host/spark_clip {bike} build/cases/{name}.hex --json {flags}"
    finished = subprocess.run(["wsl", "-e", "bash", "-lc", command], capture_output=True, text=True)
    return json.loads(finished.stdout)


def shifted_catalog(directory):
    text = (ROOT / "data" / "bikes.txt").read_text(encoding="utf-8")
    lines = []
    for line in text.split("\n"):
        if line.startswith("friction_nm"):
            value = float(line.split("=")[1])
            line = f"friction_nm = {value * (1 + FRICTION_ERROR):.4f}"
        lines.append(line)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "bikes.txt").write_text("\n".join(lines), encoding="utf-8")
    (directory / "air.txt").write_text((ROOT / "data" / "air.txt").read_text(encoding="utf-8"), encoding="utf-8")


def main():
    cases = load(CASES_FILE)["cases"]
    chosen = [name for name, case in cases.items() if not case["host"]["refused"] and case["host"]["trapped_now"] and case["labels"].get("plug") == "new" and "special" not in case["labels"]]
    shifted = BUILD / "friction_plus"
    shifted_catalog(shifted)

    def compare(name):
        bike = cases[name]["labels"]["bike"]
        both = run(name, bike, f"--data build/cases/{shifted.name}")
        timing = run(name, bike, f"--data build/cases/{shifted.name} --no-ratio")
        return cases[name]["truth"]["percent_of_new"], both["percent_of_new"]["value"], timing["percent_of_new"]["value"]

    with ThreadPoolExecutor(WORKERS) as pool:
        rows = np.array(list(pool.map(compare, chosen)))
    print(f"catalog friction off by +{FRICTION_ERROR * 100:.0f}%, {len(rows)} cases, compression as % of new")
    print(f"  timing and kV ratio: mean error {np.mean(rows[:, 1] - rows[:, 0]):+5.1f}  worst {np.max(np.abs(rows[:, 1] - rows[:, 0])):4.1f}")
    print(f"  timing only:         mean error {np.mean(rows[:, 2] - rows[:, 0]):+5.1f}  worst {np.max(np.abs(rows[:, 2] - rows[:, 0])):4.1f}")


if __name__ == "__main__":
    main()