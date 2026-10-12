"""Per-stage timing of a processing cycle, summarised in the log."""

import collections
import time


class CycleTimer:
    """
    Time spent per processing cycle, by stage, summarised in one log line every
    `period` seconds (wall clock): cycles per second and the mean milliseconds
    of each stage. `start()` opens a cycle, `mark(stage)` closes the stage
    running since the previous mark, `end()` closes the cycle.
    """

    def __init__(self, logger, label: str, period: float = 30.0):
        self._log, self._label, self._period = logger, label, period
        self._sums = collections.OrderedDict()
        self._cycles, self._total, self._worst = 0, 0.0, 0.0
        self._since = time.monotonic()
        self._t0 = self._last = None

    def start(self) -> None:
        self._t0 = self._last = time.perf_counter()

    def mark(self, stage: str) -> None:
        if self._last is None:
            return
        now = time.perf_counter()
        self._sums[stage] = self._sums.get(stage, 0.0) + (now - self._last)
        self._last = now

    def end(self) -> None:
        if self._t0 is None:
            return
        spent = time.perf_counter() - self._t0
        self._t0 = self._last = None
        self._cycles += 1
        self._total += spent
        self._worst = max(self._worst, spent)
        elapsed = time.monotonic() - self._since
        if elapsed >= self._period:
            n = self._cycles
            stages = ', '.join(f'{k} {1000 * v / n:.0f}' for k, v in self._sums.items())
            self._log.info(f'{self._label}: {n / elapsed:.2f} cycles/s, {1000 * self._total / n:.0f} ms per '
                           f'cycle (worst {1000 * self._worst:.0f})' + (f' | ms: {stages}' if stages else ''))
            self._sums.clear()
            self._cycles, self._total, self._worst = 0, 0.0, 0.0
            self._since = time.monotonic()
