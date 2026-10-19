import json
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from model.capture import run_capture
from physics.catalog import load_bikes
from physics.cranking import Battery, CrankingModel, Cylinder, Firing, simulate
from physics.scenario import Scenario, build_capture

BUILD = ROOT / "build" / "cases"
OUTPUT = ROOT / "web" / "data" / "cases.js"
BIKES = ("cub110", "dc110", "fi125")
BATTERIES = {"good": (12.8, 0.015), "tired": (12.5, 0.028), "weak": (12.3, 0.04)}
LEAKS_MM2 = {"sealed": 0.02, "slight": 0.1, "worn": 0.35, "bad": 0.9}
WORN_PLUG_EXTRA = 0.4e-3
NEW_ENGINE_LEAK = 0.02e-6
NOMINAL_HEAT = 60.0
REFERENCE_BATTERY = Battery(12.8, 0.015)
REFERENCE_START_DEG = 100.0
FIRING = Firing(first_compression=4, heat_release=150.0)
WORKERS = 6


def case_list(bikes):
    cases = []
    seed = 100
    for bike in BIKES:
        spec_gap = bikes[bike].ignition.plug_gap
        for battery_name, (emf, resistance) in BATTERIES.items():
            for leak_name, leak in LEAKS_MM2.items():
                for plug_name, gap in (("new", spec_gap), ("worn", spec_gap + WORN_PLUG_EXTRA)):
                    seed += 1
                    labels = {"bike": bike, "battery": battery_name, "leak": leak_name, "plug": plug_name}
                    cases.append((labels, Scenario(bike, Battery(emf, resistance), leak * 1e-6, gap, seed=seed)))
    spec_gap = bikes["cub110"].ignition.plug_gap
    cases.append(({"bike": "cub110", "battery": "good", "leak": "sealed", "plug": "new", "special": "fires"},
                  Scenario("cub110", Battery(*BATTERIES["good"]), LEAKS_MM2["sealed"] * 1e-6, spec_gap, firing=FIRING, seed=901)))
    cases.append(({"bike": "cop150", "battery": "good", "leak": "sealed", "plug": "new", "special": "coil_on_plug"},
                  Scenario("cop150", Battery(*BATTERIES["good"]), LEAKS_MM2["sealed"] * 1e-6, bikes["cop150"].ignition.plug_gap, seed=902)))
    return cases


def case_id(labels):
    return labels.get("special") or f"{labels['bike']}-{labels['battery']}-{labels['leak']}-{labels['plug']}"


def gauge(bike, leak, heat):
    model = CrankingModel(bike, REFERENCE_BATTERY, Cylinder(leak, heat))
    record = simulate(model, np.radians(REFERENCE_START_DEG), np.random.default_rng(0), jitter_deg=0.0)
    return (np.mean(record.peak_pressure[-2:]) - 1.013e5) / 1e5


def truth_of(capture, catalog_bike):
    record = capture.record
    times = np.array(record.trace_time)
    sparks = record.sparks
    first_exhaust = next(i for i, s in enumerate(sparks) if not s.compression)
    window = (times >= sparks[first_exhaust].time) & (times <= sparks[-1].time)
    scenario = capture.scenario
    new_gauge = gauge(catalog_bike, NEW_ENGINE_LEAK, NOMINAL_HEAT)
    true_gauge = gauge(capture.truth_bike, scenario.leak_area, capture.heat_transfer)
    misfires = sum(1 for s, fired in zip(sparks, capture.fired) if s.compression and not fired)
    return {
        "cranking_volts": float(np.mean(np.array(record.trace_battery)[window])),
        "leak_mm2": scenario.leak_area * 1e6,
        "percent_of_new": float(100 * true_gauge / new_gauge),
        "trapped": float(100 * np.mean(record.trapped)) if record.trapped else None,
        "plug_gap_mm": scenario.plug_gap * 1e3,
        "misfires": misfires,
        "compression_sparks": sum(1 for s in sparks if s.compression),
        "reserve": [round(o / b, 3) for s, o, b in zip(sparks, capture.open_kv, capture.breakdown_kv) if s.compression],
        "polytropic": float(np.mean(record.polytropic)) if record.polytropic else None,
    }


def record_bytes(records):
    payload = bytearray()
    for record in records:
        for word in record.words():
            payload += word.to_bytes(4, "little")
    return "\n".join(f"{b:02x}" for b in payload)


def build_one(item):
    labels, scenario = item
    name = case_id(labels)
    if (BUILD / f"{name}.meta").exists() and (BUILD / f"{name}.hex").exists():
        return name
    bikes = load_bikes()
    capture = build_capture(scenario, bikes)
    records = run_capture(capture.calibration, capture.windows)
    (BUILD / f"{name}.hex").write_text(record_bytes(records), encoding="ascii")
    meta = {"labels": labels, "truth": truth_of(capture, bikes[scenario.bike])}
    (BUILD / f"{name}.meta").write_text(json.dumps(meta), encoding="utf-8")
    return name


def wsl_root():
    text = str(ROOT).replace("\\", "/")
    return "/mnt/" + text[0].lower() + text[2:]


def host_done(name):
    output = BUILD / f"{name}.json"
    return output.exists() and output.stat().st_size > 0


def run_host(names, bikes_by_name):
    lines = [f"host/spark_clip {bikes_by_name[n]} build/cases/{n}.hex --json > build/cases/{n}.json 2> build/cases/{n}.err" for n in names if not host_done(n)]
    (BUILD / "run_host.sh").write_bytes(("\n".join(lines) + "\n").encode("ascii"))
    command = f"cd '{wsl_root()}' && make -C host spark_clip >/dev/null && xargs -d '\\n' -P {WORKERS} -I{{}} sh -c '{{}}' < build/cases/run_host.sh"
    subprocess.run(["wsl", "-e", "bash", "-lc", command], capture_output=True, text=True)
    failed = [f"{name}: {(BUILD / f'{name}.err').read_text().strip()}" for name in names if (BUILD / f"{name}.err").read_text().strip()]
    if failed:
        raise RuntimeError("host failed\n" + "\n".join(failed))


def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    bikes = load_bikes()
    with ProcessPoolExecutor(WORKERS) as pool:
        names = list(pool.map(build_one, case_list(bikes)))
    built = {name: json.loads((BUILD / f"{name}.meta").read_text(encoding="utf-8")) for name in names}
    run_host(names, {name: built[name]["labels"]["bike"] for name in names})
    for name in names:
        built[name]["host"] = json.loads((BUILD / f"{name}.json").read_text(encoding="utf-8"))
    bikes_meta = {name: {"label": bike.label, "ignition": bike.ignition.kind, "plug_gap_mm": bike.ignition.plug_gap * 1e3} for name, bike in bikes.items()}
    OUTPUT.write_text("window.SPARK_CASES = " + json.dumps({"bikes": bikes_meta, "cases": built}, separators=(",", ":")) + ";", encoding="utf-8")
    print(f"cases {len(names)}  written {OUTPUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()