import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from model.capture import SparkRecord, parse_words, run_capture
from model import rtl_check
from physics.catalog import load_bikes
from physics.cranking import Battery
from physics.scenario import Scenario, build_capture


def test_words_round_trip():
    records = [SparkRecord(1234, 900, 410, 15000, True, [0, 12, 30]), SparkRecord(99999, 210, 70, 0, False, [0, 5])]
    words = [word for record in records for word in record.words()]
    assert parse_words(words) == records


def test_capture_finds_every_spark():
    scenario = Scenario("cub110", Battery(12.8, 0.015), 0.02e-6, 0.7e-3, seed=21)
    capture = build_capture(scenario, load_bikes())
    records = run_capture(capture.calibration, capture.windows)
    assert len(records) == len(capture.record.sparks)
    assert [r.sparked for r in records] == capture.fired
    assert all(abs(r.start_time - w.start) < 2000 for r, w in zip(records, capture.windows))


def test_compression_peaks_stand_above_exhaust():
    scenario = Scenario("fi125", Battery(12.8, 0.015), 0.02e-6, 0.8e-3, seed=22)
    capture = build_capture(scenario, load_bikes())
    records = run_capture(capture.calibration, capture.windows)
    compression = [r.peak_height for r, s in zip(records, capture.record.sparks) if s.compression]
    exhaust = [r.peak_height for r, s in zip(records, capture.record.sparks) if not s.compression]
    assert min(compression) > 3 * max(exhaust)


@pytest.mark.slow
def test_rtl_matches_model_bit_for_bit():
    assert rtl_check.check_capture() == 0
    assert rtl_check.check_top() == 0