#!/usr/bin/env python3
"""
Activity 1 - Fault-tolerant CAN network simulation for a smart vehicle
======================================================================

Covers all six required tasks:
  1. Bit-wise CAN arbitration and identifier-based message ordering
  2. Frame transmission duration and cumulative waiting times
  3. 2 ms deadline verification for safety-critical frames
  4. Single transmission error and automatic retransmission
  5. Trade-off analysis: is a higher bit rate always better?
  6. Recommended network and fault-tolerance strategy

Includes extensions:
  - Q1: Priority inversion when infotainment outranks ABS/Airbag
  - Q3: Periodic timeline simulation with random bit errors, bus load,
        TEC fault confinement, error-passive penalty, and bus-off
  - Q4: Worst-case response-time analysis (Tindell / Davis style)

Frame duration model:
    T_frame = (47 + 8 * DLC) * 1.2 / bitrate

Run with defaults:
    python can_sim.py
Run with custom parameters:
    python can_sim.py --bitrate 250000 --length 40 --seed 42
"""

from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Configuration & Data Models
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class NetworkConfig:
    bitrate: float = 500_000.0       # nominal bit rate (bit/s)
    cable_length: float = 40.0       # bus length (m)
    critical_deadline: float = 2e-3  # deadline for safety-critical frames (s)

    # Frame timing approximation
    fixed_overhead_bits: int = 47
    stuff_factor: float = 1.2

    # ISO 11898 fault-confinement model (simplified)
    error_frame_bits: int = 12 + 8 + 3     # flag (worst-case 12) + delim (8) + intermission (3)
    suspend_tx_bits: int = 8               # extra delay for error-passive nodes
    tec_error_inc: int = 8
    tec_success_dec: int = 1
    error_passive_limit: int = 128
    bus_off_limit: int = 256
    bus_off_recovery_bits: int = 128 * 11  # 128 occurrences of 11 recessive bits

    # Physical-layer timing assumptions
    prop_delay_per_m: float = 5e-9         # ~5 ns/m on twisted pair
    node_loop_delay: float = 150e-9        # controller + transceiver delay (s)
    prop_seg_fraction: float = 0.70        # share of bit time for propagation
    length_margin_req: float = 0.25        # minimum 25% cable-length margin


@dataclass(frozen=True)
class Message:
    ecu: str
    name: str
    can_id: int      # 11-bit standard identifier
    dlc: int         # payload size in bytes (0..8)
    period: float    # release period (s)
    critical: bool

    def deadline(self, cfg: NetworkConfig) -> float:
        return cfg.critical_deadline if self.critical else self.period

    @property
    def label(self) -> str:
        return f"{self.ecu} (0x{self.can_id:03X})"


@dataclass
class TxResult:
    msg: Message
    release_time: float
    start_time: float
    finish_time: float
    attempts: int


@dataclass
class MessageStats:
    sent: int = 0
    max_resp: float = 0.0
    misses: int = 0
    errors: int = 0
    overwritten: int = 0
    dropped: int = 0


DEFAULT_MESSAGES: list[Message] = [
    Message("ABS",       "Emergency braking", 0x080, 8, 10e-3, True),
    Message("Airbag",    "Crash status",      0x040, 8, 20e-3, True),
    Message("Engine",    "Engine speed",      0x180, 8, 10e-3, False),
    Message("Dashboard", "Vehicle speed",     0x300, 8, 50e-3, False),
]


# ---------------------------------------------------------------------------
# Formatting & Timing Helpers
# ---------------------------------------------------------------------------
def fmt_us(seconds: float) -> str:
    if math.isinf(seconds):
        return "     inf"
    return f"{seconds * 1e6:8.1f} us"


def frame_time(dlc: int, bitrate: float, cfg: NetworkConfig) -> float:
    return (cfg.fixed_overhead_bits + 8 * dlc) * cfg.stuff_factor / bitrate


def error_overhead(bitrate: float, cfg: NetworkConfig) -> float:
    return cfg.error_frame_bits / bitrate


def bus_load(msgs: list[Message], bitrate: float, cfg: NetworkConfig) -> float:
    return sum(frame_time(m.dlc, bitrate, cfg) / m.period for m in msgs)


def print_section(title: str) -> None:
    print("\n" + "=" * 80)
    print(title)
    print("=" * 80)


# ---------------------------------------------------------------------------
# Task 1 - Bit-wise CAN Arbitration
# ---------------------------------------------------------------------------
def arbitrate(contenders: list[Message], trace: bool = False) -> Message:
    """
    Simulate 11-bit CAN arbitration (MSB to LSB) using wired-AND bus logic.
    A dominant '0' overrides a recessive '1'; losing nodes immediately back off.
    """
    if len({m.can_id for m in contenders}) != len(contenders):
        raise ValueError("Duplicate CAN identifiers detected on the bus.")

    active = list(contenders)
    for bit_pos in range(10, -1, -1):
        bus_level = min((m.can_id >> bit_pos) & 1 for m in active)
        losers = [m for m in active if ((m.can_id >> bit_pos) & 1) != bus_level]

        if losers and trace:
            level_str = "dominant(0)" if bus_level == 0 else "recessive(1)"
            loser_names = ", ".join(m.ecu for m in losers)
            print(f"    Bit {bit_pos:2d}: bus={level_str} -> {loser_names} back off")

        active = [m for m in active if ((m.can_id >> bit_pos) & 1) == bus_level]
        if len(active) == 1:
            break

    return active[0]


def arbitration_order(msgs: list[Message], trace: bool = False) -> list[Message]:
    remaining = list(msgs)
    order: list[Message] = []
    round_num = 1

    while remaining:
        if trace:
            print(f"  Round {round_num}: contenders = {[m.ecu for m in remaining]}")
        winner = arbitrate(remaining, trace=trace)
        if trace:
            print(f"    -> winner: {winner.label}")
        order.append(winner)
        remaining.remove(winner)
        round_num += 1

    return order


# ---------------------------------------------------------------------------
# Tasks 2-4 - Simultaneous Burst Simulation (t = 0)
# ---------------------------------------------------------------------------
def simulate_burst(
    msgs: list[Message],
    bitrate: float,
    cfg: NetworkConfig,
    errors: dict[str, int] | None = None,
    log: bool = False,
) -> list[TxResult]:
    """
    Simulate all messages becoming ready at t = 0.
    `errors` maps ECU name -> number of transmission attempts to corrupt.
    """
    remaining_errors = dict(errors or {})
    pending = list(msgs)
    attempts = {m: 0 for m in msgs}
    results: list[TxResult] = []
    now = 0.0

    while pending:
        winner = arbitrate(pending)
        tx_dur = frame_time(winner.dlc, bitrate, cfg)
        attempts[winner] += 1

        if remaining_errors.get(winner.ecu, 0) > 0:
            remaining_errors[winner.ecu] -= 1
            if log:
                print(
                    f"  t={fmt_us(now)}  {winner.label:18s} ERROR detected "
                    f"-> error frame ({cfg.error_frame_bits} bits), retrying"
                )
            now += tx_dur + error_overhead(bitrate, cfg)
            continue

        finish = now + tx_dur
        if log:
            print(
                f"  t={fmt_us(now)}  {winner.label:18s} OK "
                f"(attempt {attempts[winner]}), finishes at {fmt_us(finish).strip()}"
            )
        results.append(TxResult(winner, 0.0, now, finish, attempts[winner]))
        now = finish
        pending.remove(winner)

    return results


def print_burst_table(
    results: list[TxResult],
    bitrate: float,
    cfg: NetworkConfig,
) -> None:
    print(
        f"  {'#':>2} {'Message':18s} {'Tx time':>11} {'Wait':>11} "
        f"{'Finish':>11} {'Deadline':>10} {'Tries':>5}  Status"
    )
    total_wait = 0.0
    for idx, res in enumerate(results, 1):
        tx_dur = frame_time(res.msg.dlc, bitrate, cfg)
        wait = res.start_time - res.release_time
        total_wait += wait
        dl = res.msg.deadline(cfg)
        ok = res.finish_time <= dl
        status = ("MEETS" if ok else "MISSES") + (" (critical)" if res.msg.critical else "")
        print(
            f"  {idx:>2} {res.msg.label:18s} {fmt_us(tx_dur)} {fmt_us(wait)} "
            f"{fmt_us(res.finish_time)} {dl * 1e3:7.1f} ms {res.attempts:>5}  {status}"
        )
    print(
        f"  Cumulative wait: {fmt_us(total_wait).strip()} | "
        f"Bus busy until: {fmt_us(results[-1].finish_time).strip()}"
    )


# ---------------------------------------------------------------------------
# Q4 - Worst-Case Response-Time Analysis (Tindell / Davis style)
# ---------------------------------------------------------------------------
def response_time_analysis(
    msgs: list[Message],
    bitrate: float,
    cfg: NetworkConfig,
    n_errors: int = 1,
) -> dict[Message, float]:
    """
    Compute worst-case response time R = B + I + E + C for each message:
      B: non-preemptive blocking from one lower-priority frame
      I: interference from higher-priority frames
      E: recovery overhead for `n_errors` bus errors
      C: own transmission time
    """
    t_bit = 1.0 / bitrate
    ordered = sorted(msgs, key=lambda m: m.can_id)
    wcrt: dict[Message, float] = {}

    for idx, msg in enumerate(ordered):
        tx_dur = frame_time(msg.dlc, bitrate, cfg)
        higher_prio = ordered[:idx]
        lower_prio = ordered[idx + 1:]

        blocking = max((frame_time(x.dlc, bitrate, cfg) for x in lower_prio), default=0