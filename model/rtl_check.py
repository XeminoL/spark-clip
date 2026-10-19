import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from model.capture import HOLDOFF, CaptureModel, parse_words
from physics.catalog import load_bikes
from physics.cranking import Battery
from physics.scenario import Scenario, build_capture

BUILD = ROOT / "build" / "rtl"
WINDOWS_PER_CASE = 6
PADDING = HOLDOFF + 2000
CASES = {
    "cdi_healthy": Scenario("cub110", Battery(12.8, 0.015), 0.02e-6, 0.7e-3, seed=11),
    "cdi_misfire": Scenario("cub110", Battery(12.3, 0.04), 0.02e-6, 1.1e-3, seed=12),
    "tci_worn": Scenario("fi125", Battery(12.5, 0.028), 0.25e-6, 0.9e-3, seed=13),
}
CAPTURE_SOURCES = ["rtl/spark_capture.v", "rtl/record_packer.v", "tb/spark_capture_tb.v"]
TOP_SOURCES = ["rtl/spark_capture.v", "rtl/record_packer.v", "rtl/async_fifo.v", "rtl/uart_tx.v", "rtl/spark_clip_top.v", "tb/spark_clip_top_tb.v"]
TOP_CASE = "cdi_healthy"
LEAD_IN_SEARCH = 4


def wsl_path(path):
    text = str(path).replace("\\", "/")
    return "/mnt/" + text[0].lower() + text[2:]


def build_stream(capture, rng):
    pieces = [capture.calibration]
    for window in capture.windows[:WINDOWS_PER_CASE]:
        pieces.append(rng.choice(capture.calibration, PADDING))
        pieces.append(window.samples)
    pieces.append(rng.choice(capture.calibration, PADDING))
    return np.concatenate(pieces).astype(np.int16)


def write_stimulus(stream, path):
    path.write_text("\n".join(f"{int(value) & 0xFFF:03x}" for value in stream), encoding="ascii")


def model_records(stream):
    model = CaptureModel()
    model.feed(model.calibrate(stream))
    return model.records


def run_simulation(sources, top, plusargs, workdir):
    binary = wsl_path(workdir / f"{top}.vvp")
    files = " ".join(f"'{wsl_path(ROOT / source)}'" for source in sources)
    arguments = " ".join(f"'+{key}={wsl_path(value)}'" for key, value in plusargs.items())
    command = f"iverilog -g2012 -o '{binary}' -s {top} {files} && vvp -n '{binary}' {arguments}"
    finished = subprocess.run(["wsl", "-e", "bash", "-lc", command], capture_output=True, text=True)
    if finished.returncode != 0:
        raise RuntimeError(finished.stdout + finished.stderr)
    return finished.stdout.strip()


def read_bytes_as_words(path):
    values = [int(line, 16) for line in path.read_text(encoding="ascii").split()]
    if len(values) % 4:
        raise ValueError("partial word")
    return [values[i] | values[i + 1] << 8 | values[i + 2] << 16 | values[i + 3] << 24 for i in range(0, len(values), 4)]


def compare(name, expected, actual):
    expected_words = [word for record in expected for word in record.words()]
    actual_words = [word for record in actual for word in record.words()]
    mismatches = sum(a != b for a, b in zip(expected_words, actual_words)) + abs(len(expected_words) - len(actual_words))
    sparks = sum(record.sparked for record in expected)
    print(f"{name:12s} records {len(expected):2d}/{len(actual):2d}  sparked {sparks:2d}  words {len(expected_words):4d}  mismatches {mismatches}")
    return mismatches


def check_capture():
    BUILD.mkdir(parents=True, exist_ok=True)
    bikes = load_bikes()
    failures = 0
    for name, scenario in CASES.items():
        rng = np.random.default_rng(scenario.seed)
        stream = build_stream(build_capture(scenario, bikes), rng)
        stimulus = BUILD / f"{name}_stimulus.hex"
        output = BUILD / f"{name}_rtl.hex"
        write_stimulus(stream, stimulus)
        run_simulation(CAPTURE_SOURCES, "spark_capture_tb", {"stimulus": stimulus, "output": output}, BUILD)
        failures += compare(name, model_records(stream), parse_words(read_bytes_as_words(output)))
    return failures


def check_top():
    scenario = CASES[TOP_CASE]
    rng = np.random.default_rng(scenario.seed)
    stream = build_stream(build_capture(scenario, load_bikes()), rng)
    stimulus = BUILD / "top_stimulus.hex"
    output = BUILD / "top_uart.hex"
    write_stimulus(stream, stimulus)
    run_simulation(TOP_SOURCES, "spark_clip_top_tb", {"stimulus": stimulus, "output": output}, BUILD)
    received = parse_words(read_bytes_as_words(output))
    received_words = [word for record in received for word in record.words()]
    for lead_in in range(LEAD_IN_SEARCH):
        expected = model_records(np.concatenate([np.zeros(lead_in, dtype=np.int16), stream]))
        if [word for record in expected for word in record.words()] == received_words:
            return compare(f"top lead {lead_in}", expected, received)
    return compare("top", model_records(stream), received)


if __name__ == "__main__":
    sys.exit(1 if check_capture() + check_top() else 0)