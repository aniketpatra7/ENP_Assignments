#!/usr/bin/env python3
"""
CAN Hardware Architecture & SJA1000 Acceptance Filter Checker
=============================================================
Evaluates:
  1. External vs. internal CAN controller architecture
  2. Basic CAN vs. Full CAN (PeliCAN) mode selection
  3. SJA1000 8-bit acceptance filter match/reject logic
  4. Standard CAN payload length validation (0-8 bytes)
"""

from __future__ import annotations

from dataclasses import dataclass

FULL_CAN_ID_THRESHOLD = 15
MAX_STANDARD_CAN_ID = 0x7FF
MAX_STANDARD_DLC = 8


@dataclass(frozen=True)
class CANHardwareConfig:
    mc_has_can: bool
    num_identifiers: int
    req_id: int
    data_len: int
    acr: int
    amr: int

    def validate(self) -> None:
        if self.num_identifiers < 0:
            raise ValueError("num_identifiers cannot be negative.")
        if not (0 <= self.req_id <= MAX_STANDARD_CAN_ID):
            raise ValueError(f"req_id 0x{self.req_id:X} is outside 11-bit standard CAN range (0x000-0x7FF).")
        if not (0 <= self.acr <= 0xFF):
            raise ValueError(f"acr 0x{self.acr:X} must be an 8-bit value (0x00-0xFF).")
        if not (0 <= self.amr <= 0xFF):
            raise ValueError(f"amr 0x{self.amr:X} must be an 8-bit value (0x00-0xFF).")


def evaluate_acceptance_filter(req_id: int, acr: int, amr: int) -> tuple[bool, int, int]:
    """
    SJA1000 standard-frame single-byte filter check on ID[10:3].
    AMR bit = 1 -> don't care; AMR bit = 0 -> must match ACR.
    Returns (accepted, id_top_8_bits, relevant_mask).
    """
    id_top_8 = (req_id >> 3) & 0xFF
    relevant_mask = (~amr) & 0xFF
    accepted = (id_top_8 & relevant_mask) == (acr & relevant_mask)
    return accepted, id_top_8, relevant_mask


def check_can_architecture(cfg: CANHardwareConfig) -> None:
    cfg.validate()

    print("=== CAN Hardware Architecture Selection ===")
    if not cfg.mc_has_can:
        print(
            "Hardware Path : Microcontroller -> SPI/Parallel Bus -> "
            "SJA1000 CAN Controller -> CAN Transceiver -> CAN Bus"
        )
    else:
        print("Hardware Path : Microcontroller (Internal CAN) -> CAN Transceiver -> CAN Bus")

    if cfg.num_identifiers > FULL_CAN_ID_THRESHOLD:
        print("Controller    : FULL CAN (SJA1000 PeliCAN mode)")
        print(
            f"Reason        : {cfg.num_identifiers} identifiers (> {FULL_CAN_ID_THRESHOLD}); "
            "64-byte RX FIFO and hardware filtering prevent CPU overload."
        )
    else:
        print("Controller    : BASIC CAN")
        print(
            f"Reason        : {cfg.num_identifiers} identifiers (<= {FULL_CAN_ID_THRESHOLD}); "
            "a simple FIFO buffer is sufficient."
        )

    print("\n=== Acceptance Filter Verification ===")
    accepted, id_top_8, relevant_mask = evaluate_acceptance_filter(cfg.req_id, cfg.acr, cfg.amr)
    print(f"Required CAN ID : 0x{cfg.req_id:03X}  (11-bit: {cfg.req_id:011b})")
    print(f"ID Top 8 Bits   : 0x{id_top_8:02X}   (bin: {id_top_8:08b})")
    print(f"ACR (Code)      : 0x{cfg.acr:02X}   (bin: {cfg.acr:08b})")
    print(f"AMR (Mask)      : 0x{cfg.amr:02X}   (bin: {cfg.amr:08b})  [1 = don't care]")
    print(f"Active Mask     : 0x{relevant_mask:02X}   (bin: {relevant_mask:08b})  [1 = must match]")

    verdict = "MATCH (ACCEPT)" if accepted else "NO MATCH (REJECT)"
    print(f"Verdict         : {verdict} for message ID 0x{cfg.req_id:03X}.")

    print("\n=== Data Length Check ===")
    if 0 <= cfg.data_len <= MAX_STANDARD_DLC:
        print(f"Data Length     : {cfg.data_len} bytes -> VALID (Standard CAN supports 0-8 bytes).")
    else:
        print(f"Data Length     : {cfg.data_len} bytes -> INVALID for Standard CAN (must be 0-8 bytes).")


if __name__ == "__main__":
    config = CANHardwareConfig(
        mc_has_can=False,
        num_identifiers=32,
        req_id=0x123,
        data_len=8,
        acr=0x24,
        amr=0x03,
    )
    check_can_architecture(config)