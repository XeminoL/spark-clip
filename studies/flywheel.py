import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from physics.catalog import load_bikes
from physics.inertia import inertia_budget, rotor_by_integral, rotor_closed_form

PARTS = ("rotor", "webs", "crank_pin", "rod_big_end", "reciprocating", "other", "starter")


def main():
    for name, bike in load_bikes().items():
        integral = rotor_by_integral(bike.rotating)
        closed = rotor_closed_form(bike.rotating)
        budget = inertia_budget(bike, rotor_by_integral)
        print(f"{name}: rotor triple integral {integral * 1e3:.4f} g m2, closed form {closed * 1e3:.4f}, gap {abs(integral - closed) / closed:.1e}")
        print("  " + "  ".join(f"{part} {getattr(budget, part) * 1e3:.3f}" for part in PARTS) + f"  total {budget.total * 1e3:.3f} g m2")
        print(f"  starter armature seen through the gears: {budget.starter / budget.total * 100:.0f}% of the total")


if __name__ == "__main__":
    main()