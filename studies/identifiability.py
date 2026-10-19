import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from physics.catalog import load_bikes
from physics.cranking import Battery, CrankingModel, Cylinder, simulate

BASE = {"emf": 12.3, "resistance": 0.04, "leak": 0.25e-6, "heat": 60.0}
SPARKS_USED = 14
JITTER_DEG = 0.3
DENSITY_SCATTER = 0.06
RELATIVE_STEP = 0.01


def observe(bike, values):
    model = CrankingModel(bike, Battery(values["emf"], values["resistance"]), Cylinder(values["leak"], values["heat"]))
    record = simulate(model, np.radians(100), np.random.default_rng(1), jitter_deg=0.0)
    times = np.array([s.time for s in record.sparks])[:SPARKS_USED]
    density = np.array([s.density_ratio for s in record.sparks if s.compression])[: SPARKS_USED // 2]
    return times, density


def scaled_jacobian(bike, names, with_density):
    times, density = observe(bike, BASE)
    timing_sigma = np.radians(JITTER_DEG) / (2 * np.pi) * np.diff(np.r_[0.0, times])
    columns = []
    for name in names:
        moved = dict(BASE)
        moved[name] *= 1 + RELATIVE_STEP
        shifted_times, shifted_density = observe(bike, moved)
        column = list((shifted_times - times) / timing_sigma)
        if with_density:
            column += list((shifted_density - density) / (DENSITY_SCATTER * density))
        columns.append(np.array(column))
    return np.column_stack(columns), np.diff(times)


def main():
    bike = load_bikes()["cub110"]
    jacobian, intervals = scaled_jacobian(bike, ["emf", "resistance", "leak", "heat"], True)
    print("revolution times ms: " + " ".join(f"{t * 1e3:.1f}" for t in intervals[:6]) + " ...")
    print(f"emf and resistance columns correlate at {np.corrcoef(jacobian[:, 0], jacobian[:, 1])[0, 1]:.5f}")
    for names, with_density in ((["emf", "resistance", "leak", "heat"], True), (["emf", "leak", "heat"], True), (["emf", "leak", "heat"], False)):
        matrix, _ = scaled_jacobian(bike, names, with_density)
        singular = np.linalg.svd(matrix, compute_uv=False)
        label = ", ".join(names) + (" + kV ratio" if with_density else ", timing only")
        print(f"{label:38s} singular values " + " ".join(f"{s:9.3g}" for s in singular) + f"   1% of the weakest needs sigma x{1 / singular[-1]:.1f}")


if __name__ == "__main__":
    main()