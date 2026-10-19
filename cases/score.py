import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CASES_FILE = ROOT / "web" / "data" / "cases.js"
BATTERY_BANDS = ((10.5, "good"), (9.6, "weak"), (-1e9, "flat"))
COMPRESSION_BANDS = ((85.0, "good"), (70.0, "worn"), (-1e9, "low"))
RESERVE_MARGINAL = 1.15
MISFIRE_SHARE = 0.3


def band(value, bands):
    return next(name for limit, name in bands if value >= limit)


def truth_ignition(truth):
    if truth["misfires"] >= MISFIRE_SHARE * truth["compression_sparks"]:
        return "no spark"
    if truth["misfires"] > 0:
        return "marginal"
    reserves = sorted(truth["reserve"])
    return "marginal" if reserves[len(reserves) // 2] < RESERVE_MARGINAL else "good"


def load(path):
    text = Path(path).read_text(encoding="utf-8")
    return json.loads(text[text.index("=") + 1 : text.rindex(";")])


def score_cases(path=CASES_FILE):
    everything = load(path)["cases"]
    tally = {"right": 0, "unsure": 0, "wrong": 0, "refused": 0}
    wrong = []
    for name, case in everything.items():
        host = case["host"]
        if host["refused"]:
            tally["refused"] += 1
            continue
        truth = case["truth"]
        expected = {
            "battery": band(truth["cranking_volts"], BATTERY_BANDS),
            "compression": band(truth["percent_of_new"], COMPRESSION_BANDS),
            "ignition": truth_ignition(truth),
        }
        found = {"battery": host["battery"]["state"], "compression": host["compression"]["state"], "ignition": host["ignition_verdict"]["state"]}
        for part in expected:
            if found[part] == "unsure":
                tally["unsure"] += 1
            elif found[part] == expected[part]:
                tally["right"] += 1
            else:
                tally["wrong"] += 1
                wrong.append(f"{name} {part}: said {found[part]}, truth {expected[part]}")
    return {**tally, "details": wrong}


if __name__ == "__main__":
    summary = score_cases(sys.argv[1] if len(sys.argv) > 1 else CASES_FILE)
    print(f"right {summary['right']}  unsure {summary['unsure']}  wrong {summary['wrong']}  refused {summary['refused']}")
    for line in summary["details"]:
        print("  " + line)