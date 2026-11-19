#!/usr/bin/env python3
"""
Activity 2 - Autonomous vehicle CAN vs. FlexRay deadline analysis
=================================================================
Evaluates which messages belong on event-triggered CAN and which require
time-triggered FlexRay, verifying deadlines across normal, burst, and
failure scenarios.

Tasks covered:
  1. Transmission time of each message on CAN and on FlexRay
  2. Busy CAN bus with an emergency burst of 15 simultaneous frames
  3. Deterministic FlexRay static schedule (steering, brake, wheel speed)
  4. Worst-case latency under CAN and FlexRay models
  5. Deadline compliance across all messages
  6. Recommended hybrid architecture and rationale
Plus demos for Questions 1-4.

Run with defaults:
    python can_flexray_sim.py
Run with custom parameters:
    python can_flexray_sim.py --can-bitrate 500000 --fr-bitrate 5000000 --errors 1
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, replace


# ---------------------------------------------------------------------------
# Configuration & Data Models
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SystemConfig:
    can_bitrate: float = 500_000.0       # bit/s
    fr_bitrate: float = 5_000_000.0      # bit/s
    fr_cycle: float = 5e-3               # FlexRay communication cycle (s)
    fr_slot: float = 1e-3                # static slot duration (s)
    fr_static_slots: int = 4             # 4 x 1 ms static slots; remainder = dynamic

    # Frame & fault-modelling assumptions
    fixed_overhead_bits: int = 47
    stuff_factor: float = 1.2
    error_frame_bits: int = 23           # 12 flag + 8 delimiter + 3 IFS (worst case)
    infotainment_min_gap: float = 20e-3  # minimum inter-arrival time for events (s)
    tt_deadline_limit: float = 10e-3     # safety-critical threshold for FlexRay (s)
    n_errors: int = 1                    # assumed bus errors in worst-case window

    def validate(self) -> None:
        if self.fr_static_slots * self.fr_slot > self.fr_cycle:
            raise ValueError("FlexRay static slots exceed total cycle duration.")


@dataclass(frozen=True)
class Message:
    name: str
    can_id: int                 # 11-bit CAN ID (used when routed over CAN)
    dlc: int                    # payload length in bytes
    period: float | None        # None indicates event-driven traffic
    deadline: float             # maximum allowed latency (s)
    safety_critical: bool
    other_ecu: bool = False     # True for background traffic from other ECUs

    def min_gap(self, cfg: SystemConfig) -> float:
        return self.period if self.period is not None else cfg.infotainment_min_gap


@dataclass(frozen=True)
class BurstEntry:
    msg: Message
    start: float
    finish: float


# Primary vehicle messages (existing CAN ID plan with other ECUs holding low IDs)
BRAKE    = Message("Brake command",       0x010, 8, 5e-3,   5e-3,   True)
STEERING = Message("Steering command",    0x040, 8, 5e-3,   5e-3,   True)
WHEEL    = Message("Wheel-speed data",    0x050, 8, 10e-3,  10e-3,  True)
BATTERY  = Message("Battery status",      0x100, 8, 100e-3, 100e-3, False)
INFO     = Message("Infotainment update", 0x200, 8, None,   100e-3, False)
VEHICLE  = [STEERING, BRAKE, WHEEL, BATTERY, INFO]

# Other ECUs sharing the CAN bus during an emergency
AIRBAG = Message("Airbag pre-crash", 0x004, 8, 20e-3, 20e-3, True, other_ecu=True)
ESC    = Message("ESC yaw rate",     0x008, 8, 10e-3, 10e-3, True, other_ecu=True)


def adas_burst(count: int) -> list[Message]:
    """Generate `count` back-to-back radar/camera object frames."""
    return [
        Message(f"ADAS object #{k + 1}", 0x011 + k, 8, None, 100e-3, False, other_ecu=True)
        for k in range(count)
    ]


# ---------------------------------------------------------------------------
# Formatting & Timing Helpers
# ---------------------------------------------------------------------------
def frame_time(dlc: int, bitrate: float, cfg: SystemConfig) -> float:
    """Teaching frame-time approximation: (47 + 8 * DLC) * 1.2 / bitrate."""
    return (cfg.fixed_overhead_bits + 8 * dlc) * cfg.stuff_factor / bitrate


def fmt_ms(seconds: float) -> str:
    if math.isinf(seconds):
        return "    inf ms"
    return f"{seconds * 1e3:7.3f} ms"


def print_section(title: str) -> None:
    print("\n" + "=" * 80)
    print(title)
    print("=" * 80)


def status_tag(latency: float, deadline: float) -> str:
    return "MEETS" if latency <= deadline else "MISSES"


# ---------------------------------------------------------------------------
# CAN Models
# ---------------------------------------------------------------------------
def can_burst(msgs: list[Message], cfg: SystemConfig) -> list[BurstEntry]:
    """
    Simulate all frames becoming ready at t = 0.
    Lowest CAN ID wins each arbitration round; frames transmit back-to-back.
    """
    if len({m.can_id for m in msgs}) != len(msgs):
        raise ValueError("Duplicate CAN identifiers detected.")

    now = 0.0
    entries: list[BurstEntry] = []
    for msg in sorted(msgs, key=lambda m: m.can_id):
        tx_dur = frame_time(msg.dlc, cfg.can_bitrate, cfg)
        entries.append(BurstEntry(msg, now, now + tx_dur))
        now += tx_dur
    return entries


def can_worst_case(
    msgs: list[Message],
    cfg: SystemConfig,
    n_errors: int | None = None,
) -> dict[Message, float]:
    """
    Worst-case response time on CAN: R = B + I + E + C
      B: blocking from one lower-priority frame already on the bus
      I: interference from higher-priority frames released while waiting
      E: recovery overhead for `n_errors` corrupted frames
      C: own transmission time
    """
    err_count = cfg.n_errors if n_errors is None else n_errors
    t_bit = 1.0 / cfg.can_bitrate
    ordered = sorted(msgs, key=lambda m: m.can_id)
    wcrt: dict[Message, float] = {}

    for idx, msg in enumerate(ordered):
        tx_dur = frame_time(msg.dlc, cfg.can_bitrate, cfg)
        higher = ordered[:idx]
        lower = ordered[idx + 1:]

        blocking = max((frame_time(x.dlc, cfg.can_bitrate, cfg) for x in lower), default=0.0)
        max_retry = max(frame_time(x.dlc, cfg.can_bitrate, cfg) for x in higher + [msg])
        err_overhead = err_count * (cfg.error_frame_bits * t_bit + max_retry)

        w = blocking + err_overhead
        for _ in range(1000):
            interference = sum(
                math.ceil((w + t_bit) / hp.min_gap(cfg)) * frame_time(hp.dlc, cfg.can_bitrate, cfg)
                for hp in higher
            )
            w_next = blocking + err_overhead + interference
            if abs(w_next - w) < 1e-12:
                wcrt[msg] = w_next + tx_dur
                break
            if w_next > 100.0 * msg.deadline:
                wcrt[msg] = math.inf
                break
            w = w_next
        else:
            wcrt[msg] = math.inf

    return wcrt


def can_load(msgs: list[Message], cfg: SystemConfig) -> float:
    return sum(frame_time(m.dlc, cfg.can_bitrate, cfg) / m.min_gap(cfg) for m in msgs)


# ---------------------------------------------------------------------------
# FlexRay Model
# ---------------------------------------------------------------------------
# Static slots in each 5 ms cycle (wheel speed runs every 5 ms for extra margin).
FR_SCHEDULE: dict[int, Message | None] = {
    1: STEERING,
    2: BRAKE,
    3: WHEEL,
    4: None,  # spare slot for future expansion / redundancy
}


def fr_slot_of(msg: Message) -> int | None:
    return next((slot for slot, m in FR_SCHEDULE.