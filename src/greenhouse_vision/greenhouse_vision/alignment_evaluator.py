"""Alignment evaluator for the dock_target approach.

Same shape as the forklift project's vision_core/alignment.py: decides,
per frame, whether the robot is lined up with the detected target well
enough to call it "aligned" -- using Schmitt-trigger hysteresis per axis
(a tight tolerance to *become* aligned, a looser one to stay aligned)
plus an N-frame debounce, so the published flag doesn't chatter right at
the threshold boundary the way a single fixed tolerance would.

Input here is just (lateral_error_m, distance_error_m) -- a color-blob
detection doesn't give a yaw estimate the way a full pallet-face pose
would, so unlike the original this evaluator only tracks two axes.
"""
from dataclasses import dataclass
from typing import Optional


@dataclass
class AlignmentParameters:
    lateral_tol_enter: float = 0.02   # m -- tight tolerance to *become* aligned
    lateral_tol_stay: float = 0.04    # m -- looser tolerance to *stay* aligned
    distance_tol_enter: float = 0.03
    distance_tol_stay: float = 0.05
    debounce_frames: int = 5          # consecutive frames before a state flip commits


@dataclass
class AlignmentResult:
    aligned_raw: bool
    aligned: bool
    reason: str
    lateral_error_m: float
    distance_error_m: float


class AlignmentEvaluator:
    """Stateful evaluator. One instance per alignment attempt."""

    def __init__(self, params: Optional[AlignmentParameters] = None):
        self._params = params or AlignmentParameters()
        self._aligned_state = False
        self._pending_state: Optional[bool] = None
        self._pending_count = 0

    def reset(self):
        self._aligned_state = False
        self._pending_state = None
        self._pending_count = 0

    def evaluate(self, lateral_error_m: Optional[float], distance_error_m: Optional[float],
                 confidence: float) -> AlignmentResult:
        if lateral_error_m is None or confidence <= 0.0:
            return self._commit(False, "no_detection", 0.0, 0.0)

        p = self._params
        # Schmitt trigger: once aligned, tolerate drifting up to the wider
        # "stay" tolerance before dropping out again -- this is what stops
        # the flag chattering when the error sits right on a single fixed
        # threshold.
        tight_lat, loose_lat = p.lateral_tol_enter, p.lateral_tol_stay
        tight_dist, loose_dist = p.distance_tol_enter, p.distance_tol_stay

        lat_tol = loose_lat if self._aligned_state else tight_lat
        dist_tol = loose_dist if self._aligned_state else tight_dist

        raw = abs(lateral_error_m) <= lat_tol and abs(distance_error_m) <= dist_tol
        reason = "within_tolerance" if raw else "outside_tolerance"

        return self._commit(raw, reason, lateral_error_m, distance_error_m)

    def _commit(self, raw: bool, reason: str, lateral_error_m: float,
                distance_error_m: float) -> AlignmentResult:
        p = self._params

        if raw == self._aligned_state:
            self._pending_state = None
            self._pending_count = 0
        else:
            if self._pending_state == raw:
                self._pending_count += 1
            else:
                self._pending_state = raw
                self._pending_count = 1

            if self._pending_count >= p.debounce_frames:
                self._aligned_state = raw
                self._pending_state = None
                self._pending_count = 0

        return AlignmentResult(
            aligned_raw=raw,
            aligned=self._aligned_state,
            reason=reason,
            lateral_error_m=lateral_error_m,
            distance_error_m=distance_error_m,
        )
