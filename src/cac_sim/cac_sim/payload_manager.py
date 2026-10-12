"""Simplified excavation/payload abstraction.

The rulebook requires the rover to physically CARRY sand from the excavation
zone to the construction zone (dragging/bulldozing is disallowed) -- but
simulating granular sand physics (dig force, pile shape, spillage) is out of
scope. This is deliberately reduced to a payload counter driven by elapsed
(simulation) time; the arm's dig/dump motions in the mission state machine are
kinematic only. Nothing here means sand was actually moved.
"""
import time


class PayloadManager:

    def __init__(self, capacity: float = 1.0, excavation_rate: float = 0.25,
                 dump_rate: float = 0.5):
        self.capacity = capacity          # abstract "full load" units
        self.payload = 0.0
        self._excavation_rate = excavation_rate  # units/sec while excavating
        self._dump_rate = dump_rate               # units/sec while dumping
        self._last_tick = None

    @staticmethod
    def _clock(now):
        return time.monotonic() if now is None else now

    def start(self, now: float = None):
        """Reset the tick reference; pass the node's (sim) time in seconds."""
        self._last_tick = self._clock(now)

    def is_full(self) -> bool:
        return self.payload >= self.capacity

    def is_empty(self) -> bool:
        return self.payload <= 0.0

    def _dt(self, now):
        now = self._clock(now)
        dt = max(0.0, now - (self._last_tick if self._last_tick is not None else now))
        self._last_tick = now
        return dt

    def tick_excavate(self, now: float = None) -> float:
        """Advance simulated excavation by elapsed time; returns the new payload level."""
        self.payload = min(self.capacity, self.payload + self._excavation_rate * self._dt(now))
        return self.payload

    def tick_dump(self, now: float = None) -> float:
        self.payload = max(0.0, self.payload - self._dump_rate * self._dt(now))
        return self.payload
