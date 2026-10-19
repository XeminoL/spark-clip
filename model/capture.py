from dataclasses import dataclass, field

import numpy as np

CALIBRATION_SHIFT = 12
TRIGGER_LEVEL = 24
LADDER_SHIFT = 4
LADDER_LEVELS = 127
COLLAPSE_LIMIT = 64
RISE_LIMIT = 16383
BURN_LEVEL = 16
BURN_GAP = 64
BURN_LIMIT = 262143
HOLDOFF = 65535
RECORD_MARKER = 0xA5


@dataclass
class SparkRecord:
    start_time: int
    peak_height: int
    peak_offset: int
    burn_length: int
    sparked: bool
    ladder: list = field(default_factory=list)

    def words(self):
        header = (RECORD_MARKER << 24) | (int(self.sparked) << 23) | (len(self.ladder) << 16) | (self.peak_height & 0xFFF)
        timing = ((self.peak_offset & 0x3FFF) << 18) | (self.burn_length & 0x3FFFF)
        packed = [header, self.start_time & 0xFFFFFFFF, timing]
        for index in range(0, len(self.ladder), 2):
            low = self.ladder[index]
            high = self.ladder[index + 1] if index + 1 < len(self.ladder) else 0
            packed.append(low | (high << 16))
        return packed


def parse_words(words):
    records = []
    position = 0
    while position < len(words):
        header = words[position]
        if header >> 24 != RECORD_MARKER:
            raise ValueError(f"bad marker at word {position}")
        rungs = (header >> 16) & 0x7F
        timing = words[position + 2]
        ladder = []
        for index in range(rungs):
            word = words[position + 3 + index // 2]
            ladder.append((word >> (16 * (index % 2))) & 0xFFFF)
        records.append(SparkRecord(words[position + 1], header & 0xFFF, timing >> 18, timing & 0x3FFFF, bool((header >> 23) & 1), ladder))
        position += 3 + (rungs + 1) // 2
    return records


class CaptureModel:
    def __init__(self):
        self.offset = None
        self.now = 0
        self.hold_left = 0
        self.records = []

    def calibrate(self, samples):
        total = int(np.sum(samples[: 1 << CALIBRATION_SHIFT].astype(np.int64)))
        self.offset = total >> CALIBRATION_SHIFT
        self.now = 1 << CALIBRATION_SHIFT
        return samples[1 << CALIBRATION_SHIFT:]

    def skip_to(self, index):
        if index < self.now:
            raise ValueError("windows overlap")
        self.hold_left = max(0, self.hold_left - (index - self.now))
        self.now = index

    def feed(self, samples):
        levels = samples.astype(np.int64) - self.offset
        position = 0
        while position < levels.size:
            if self.hold_left:
                used = min(self.hold_left, levels.size - position)
                self.hold_left -= used
                position += used
                continue
            above = np.flatnonzero(levels[position:] >= TRIGGER_LEVEL)
            if above.size == 0:
                position = levels.size
                break
            position += int(above[0])
            position = self.capture(levels, position)
        self.now += levels.size

    def capture(self, levels, trigger):
        start = self.now + trigger
        peak = int(levels[trigger])
        peak_offset = 0
        ladder = [0] if peak >= (1 << LADDER_SHIFT) else []
        index = trigger
        elapsed = 0
        sparked = False
        while True:
            index += 1
            if index >= levels.size:
                raise ValueError("window ends inside a rise")
            elapsed += 1
            level = int(levels[index])
            half = peak >> 1
            collapsed = level < half
            if level > peak:
                peak = level
                peak_offset = elapsed
            if len(ladder) < LADDER_LEVELS and level >= (len(ladder) + 1) << LADDER_SHIFT:
                ladder.append(elapsed)
            if collapsed:
                sparked = elapsed - peak_offset <= COLLAPSE_LIMIT
                break
            if elapsed == RISE_LIMIT:
                break
        burn_length = 0
        end = index
        if sparked:
            burn_length, end = self.burn(levels, index + 1)
        self.records.append(SparkRecord(start, peak, peak_offset, burn_length, sparked, ladder))
        self.hold_left = HOLDOFF
        return end + 1

    def burn(self, levels, first):
        span = levels[first : first + BURN_LIMIT]
        if span.size < min(BURN_LIMIT, BURN_GAP):
            raise ValueError("window ends inside a burn")
        below = (span < BURN_LEVEL).astype(np.int64)
        runs = np.convolve(below, np.ones(BURN_GAP, dtype=np.int64), mode="valid")
        quiet_ends = np.flatnonzero(runs == BURN_GAP)
        if quiet_ends.size == 0:
            if span.size < BURN_LIMIT:
                raise ValueError("window ends inside a burn")
            last = BURN_LIMIT - 1
        else:
            last = int(quiet_ends[0]) + BURN_GAP - 1
        lit = np.flatnonzero(span[: last + 1] >= BURN_LEVEL)
        burn_length = int(lit[-1]) + 1 if lit.size else 0
        return burn_length, first + last


def run_capture(calibration, windows):
    model = CaptureModel()
    rest = model.calibrate(calibration)
    model.feed(rest)
    for window in windows:
        model.skip_to(window.start)
        model.feed(window.samples)
    return model.records