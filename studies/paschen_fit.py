import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from physics import geometry
from physics.catalog import load_bikes
from physics.paschen import fit_air, plug_kv, save_air, three_parameter_singular_values, uniform_kv

GAPS_MM = (0.6, 0.7, 0.9, 1.1)
TRAPPED_SHARES = (1.0, 0.8, 0.6)
EXHAUST_DENSITY = 0.967


def main():
    air, residuals = fit_air()
    save_air(air)
    print(f"fit to Dakin air curve: A/K {air.ionisation_over_k:.4f} 1/(cm Torr), B {air.field_constant:.2f} V/(cm Torr)")
    print(f"worst relative error over 0.03-1.5 bar cm: {np.max(np.abs(residuals)) * 100:.2f}%")
    singular = three_parameter_singular_values()
    print(f"fitting A, B and gamma together: singular values {singular[0]:.3g} {singular[1]:.3g} {singular[2]:.3g}")
    print("gamma only enters through A/ln(1+1/gamma), so only two constants are measurable")
    engine = load_bikes()["cub110"].engine
    sealed = EXHAUST_DENSITY * geometry.sealed_density_ratio(engine)
    print(f"\nplug, tip radius 1 mm, exhaust density {EXHAUST_DENSITY}, sealed compression density {sealed:.2f}")
    print(f"{'gap mm':>7} {'exhaust kV':>11} " + " ".join(f"{'ratio ' + str(int(s * 100)) + '%':>10}" for s in TRAPPED_SHARES) + f" {'uniform kV':>11}")
    for gap in GAPS_MM:
        exhaust = plug_kv(air, EXHAUST_DENSITY, gap * 1e-3)
        ratios = [plug_kv(air, sealed * share, gap * 1e-3) / exhaust for share in TRAPPED_SHARES]
        print(f"{gap:7.1f} {exhaust:11.2f} " + " ".join(f"{r:10.3f}" for r in ratios) + f" {uniform_kv(air, EXHAUST_DENSITY * gap / 10):11.2f}")


if __name__ == "__main__":
    main()