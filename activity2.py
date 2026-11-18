#!/usr/bin/env python3
"""
Activity 2 - The autonomous vehicle that must never miss a deadline
===================================================================
Decides which messages go on event-triggered CAN and which on
time-triggered FlexRay, and checks every deadline.

Tasks covered:
  1. Transmission time of each message on CAN and on FlexRay
  2. A busy CAN bus with many messages ready at the same time
  3. A fixed (deterministic) FlexRay schedule for steering, brake, wheel speed
  4. Worst-case latency under the assumed models
  5. Which messages meet their deadlines
  6. A hybrid architecture, with the reasons for each choice
Plus small demos for Questions 1-4.

Only the Python standard library is used.  Run:  python can_flexray_sim.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

# --------------------------------------------------------------------------
# Assumptions from the activity sheet
# --------------------------------------------------------------------------
CAN_BITRATE = 500_000          # bit/s
FR_BITRATE = 5_000_000         # bit/s
FR_CYCLE = 5e-3                # FlexRay communication cycle (s)
FR_SLOT = 1e-3                 # length of one static slot (s)
FR_STATIC_SLOTS = 4            # 4 x 1 ms reserved slots; last 1 ms = dynamic part

# Our own modelling assumptions (stated so they can be questioned)
ERROR_FRAME_BITS = 23          # CAN error frame, worst case (12 flag + 8 delim + 3 IFS)
INFOTAINMENT_MIN_GAP = 20e-3   # infotainment events at most once every 20 ms
TT_DEADLINE_LIMIT = 10e-3      # deadlines this tight + safety-critical -> FlexRay


def frame_time(dlc: int, bitrate: float) -> float:
    """Same teaching formula as Activity 1: (47 + 8D) * 1.2 / R."""
    return (47 + 8 * dlc) * 1.2 / bitrate


def ms(x: float) -> str:
    return f"{x * 1e3:7.3f} ms"


def header(title: str) -> None:
    print("\n" + "=" * 78 + "\n" + title + "\n" + "=" * 78)


# --------------------------------------------------------------------------
# Messages
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class Msg:
    name: str
    can_id: int                 # used only when the message is on CAN
    dlc: int                    # data bytes
    period: float | None        # None = event-driven
    deadline: float             # maximum permitted latency
    safety_critical: bool
    other_ecu: bool = False     # True = traffic from other ECUs on the same CAN bus

    @property
    def min_gap(self) -> float:
        """Shortest time between two frames (period, or assumed for events)."""
        return self.period if self.period else INFOTAINMENT_MIN_GAP


# The five messages from the activity. CAN IDs = an existing CAN ID plan
# in which other teams' ECUs (airbag, ESC, ADAS) already hold some low IDs.
BRAKE    = Msg("Brake command",       0x010, 8, 5e-3,   5e-3,   True)
STEERING = Msg("Steering command",    0x040, 8, 5e-3,   5e-3,   True)
WHEEL    = Msg("Wheel-speed data",    0x050, 8, 10e-3,  10e-3,  True)
BATTERY  = Msg("Battery status",      0x100, 8, 100e-3, 100e-3, False)
INFO     = Msg("Infotainment update", 0x200, 8, None,   100e-3, False)
VEHICLE = [STEERING, BRAKE, WHEEL, BATTERY, INFO]

# Other ECUs sharing the CAN bus (they become very active in an emergency)
AIRBAG = Msg("Airbag pre-crash", 0x004, 8, 20e-3, 20e-3, True, other_ecu=True)
ESC    = Msg("ESC yaw rate",     0x008, 8, 10e-3, 10e-3, True, other_ecu=True)


def adas_burst(n: int) -> list[Msg]:
    """n radar/camera object frames sent back-to-back during a manoeuvre."""
    return [Msg(f"ADAS object #{k + 1}", 0x011 + k, 8, None, 100e-3, False, other_ecu=True)
            for k in range(n)]


# --------------------------------------------------------------------------
# CAN models
# --------------------------------------------------------------------------
def can_burst(msgs: list[Msg]) -> list[tuple[Msg, float, float]]:
    """All frames ready at t = 0. Lowest ID wins each arbitration,
    so frames go out in ID order, one after another.
    Returns (message, start, finish) for each frame."""
    if len({m.can_id for m in msgs}) != len(msgs):
        raise ValueError("duplicate CAN IDs")
    t, out = 0.0, []
    for m in sorted(msgs, key=lambda m: m.can_id):
        c = frame_time(m.dlc, CAN_BITRATE)
        out.append((m, t, t + c))
        t += c
    return out


def can_worst_case(msgs: list[Msg], n_errors: int = 1) -> dict[Msg, float]:
    """Worst-case latency on CAN:  R = B + I + E + C
       B = one lower-priority frame already on the bus (can't be interrupted)
       I = higher-priority frames released while waiting
       E = n_errors x (error frame + one retransmitted frame)
       C = own frame time"""
    tbit = 1 / CAN_BITRATE
    ordered = sorted(msgs, key=lambda m: m.can_id)
    out = {}
    for i, m in enumerate(ordered):
        C = frame_time(m.dlc, CAN_BITRATE)
        higher, lower = ordered[:i], ordered[i + 1:]
        B = max((frame_time(x.dlc, CAN_BITRATE) for x in lower), default=0.0)
        E = n_errors * (ERROR_FRAME_BITS * tbit
                        + max(frame_time(x.dlc, CAN_BITRATE) for x in higher + [m]))
        w = B + E
        for _ in range(1000):                       # repeat until w stops changing
            w_new = B + E + sum(math.ceil((w + tbit) / x.min_gap)
                                * frame_time(x.dlc, CAN_BITRATE) for x in higher)
            if abs(w_new - w) < 1e-12:
                break
            w = w_new
        out[m] = w + C
    return out


def can_load(msgs: list[Msg]) -> float:
    return sum(frame_time(m.dlc, CAN_BITRATE) / m.min_gap for m in msgs)


# --------------------------------------------------------------------------
# FlexRay model
# --------------------------------------------------------------------------
# Static slots in every 5 ms cycle. Wheel speed is sent every cycle (5 ms),
# which is faster than it needs (10 ms) and gives it extra margin.
FR_SCHEDULE = {1: STEERING, 2: BRAKE, 3: WHEEL, 4: None}   # slot 4 = spare


def fr_slot_of(m: Msg) -> int | None:
    return next((s for s, x in FR_SCHEDULE.items() if x == m), None)


def flexray_latency(m: Msg, synchronised: bool) -> float:
    """
    synchronised  : the sending software is locked to the FlexRay clock and
                    writes its data just before its own slot -> only the
                    frame time is left. Same value every cycle.
    unsynchronised: data can become ready just AFTER its slot started, so it
                    waits one whole cycle for the slot to come round again.
    """
    tx = frame_time(m.dlc, FR_BITRATE)
    return tx if synchronised else FR_CYCLE + tx


def print_flexray_cycle() -> None:
    for s in range(1, FR_STATIC_SLOTS + 1):
        m = FR_SCHEDULE[s]
        start = (s - 1) * FR_SLOT
        name = m.name if m else "spare (growth / redundancy)"
        print(f"  {start * 1e3:3.0f}-{(start + FR_SLOT) * 1e3:.0f} ms  static slot {s}: {name}")
    print(f"  {FR_STATIC_SLOTS:3.0f}-{FR_CYCLE * 1e3:.0f} ms  dynamic segment (event traffic) + idle time")
    print("  ... then the same pattern repeats every 5 ms, forever.")


# --------------------------------------------------------------------------
# Which network? (Task 6 decision rule)
# --------------------------------------------------------------------------
def choose_network(m: Msg) -> tuple[str, str]:
    if m.period is None:
        return "CAN", "event-driven: a reserved slot would sit empty most of the time"
    if m.safety_critical and m.deadline <= TT_DEADLINE_LIMIT:
        return "FlexRay", "safety-critical, periodic, deadline <= 10 ms -> needs a guaranteed slot"
    return "CAN", "relaxed deadline; CAN's worst case is far below it"


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def main() -> None:
    can_c = frame_time(8, CAN_BITRATE)
    fr_c = frame_time(8, FR_BITRATE)

    header("TASK 1 - Transmission time of one 8-byte frame")
    print(f"  Bits per frame = (47 + 8*8) * 1.2 = {(47 + 64) * 1.2:.1f}")
    print(f"  CAN     @ 500 kbit/s: 133.2 / 500 000   = {ms(can_c)}")
    print(f"  FlexRay @   5 Mbit/s: 133.2 / 5 000 000 = {ms(fr_c)}  (10x faster)")
    print("  All five messages are 8 bytes, so every message has these same times.")

    header("TASK 2 - Busy CAN bus: emergency manoeuvre, 15 frames ready at t = 0")
    busy = VEHICLE + [AIRBAG, ESC] + adas_burst(8)
    print(f"  {'#':>2}  {'Message':22s} {'ID':>6} {'Starts':>10} {'Arrives':>10} {'Deadline':>10}")
    for i, (m, s, f) in enumerate(can_burst(busy), 1):
        mark = "" if m.other_ecu else ("  MEETS" if f <= m.deadline else "  MISSES")
        print(f"  {i:>2}  {m.name:22s} 0x{m.can_id:03X} {ms(s)} {ms(f)} {ms(m.deadline)}{mark}")
    print("\n  Steering waits behind 11 frames: about 3 ms of delay before it even starts.")

    print("\n  What if the ADAS burst is bigger? Steering latency vs burst size:")
    print(f"  {'ADAS frames':>12} {'CAN steering':>13} {'FlexRay steering':>17}")
    for n in [0, 4, 8, 12, 15, 20]:
        res = {m: f for m, _, f in can_burst(VEHICLE + [AIRBAG, ESC] + adas_burst(n))}
        r = res[STEERING]
        flag = "MEETS" if r <= STEERING.deadline else "MISSES"
        print(f"  {n:>12} {ms(r)} {flag:>6}   {ms(flexray_latency(STEERING, True))} (always)")
    print("  CAN latency grows with other traffic; a FlexRay slot is unaffected by it.")

    header("TASK 3 - Deterministic FlexRay schedule (one 5 ms cycle)")
    print_flexray_cycle()

    header("TASKS 4 & 5 - Worst-case latency and deadlines")
    wc = can_worst_case(VEHICLE, n_errors=1)
    busy_res = {m: f for m, _, f in can_burst(busy)}
    print("  CAN worst case = these 5 messages only, any timing, 1 error allowed.")
    print("  CAN busy       = the 15-frame emergency burst from Task 2.")
    print(f"  {'Message':22s} {'Deadline':>10} {'CAN worst':>10} {'CAN busy':>10} "
          f"{'FR sync':>10} {'FR unsync':>10}")
    for m in VEHICLE:
        if fr_slot_of(m):
            fs, fu = ms(flexray_latency(m, True)), ms(flexray_latency(m, False))
        else:
            fs = fu = f"{'-':>10}"
        print(f"  {m.name:22s} {ms(m.deadline)} {ms(wc[m])} {ms(busy_res[m])} {fs} {fu}")
    print(f"\n  CAN load from these 5 messages: {can_load(VEHICLE):.1%}")
    print("  * On CAN everything meets its deadline - but only while the other traffic\n"
          "    stays as assumed. More traffic or more errors and steering can miss.")
    print("  * FlexRay, unsynchronised: steering/brake just miss (5.027 > 5 ms) - the\n"
          "    data can arrive just after its slot and wait a full cycle.")
    print("  * FlexRay, synchronised to the bus clock (how it is meant to be used):\n"
          "    0.027 ms, the same every cycle, whatever else is happening.")

    header("TASK 6 - Hybrid architecture")
    for m in VEHICLE:
        net, why = choose_network(m)
        print(f"  {m.name:22s} -> {net:8s} {why}")
    print("\n  FlexRay (channels A + B): steering, brake, wheel speed in static slots 1-3")
    print("  CAN 500 kbit/s          : battery status, infotainment (+ backup for safety msgs)")
    print("  Gateway ECU between the two networks")

    header("Q1 - Give steering the top CAN ID (0x001). Does that fix everything?")
    st_top = replace(STEERING, can_id=0x001)
    before = {m.name: f for m, _, f in can_burst(busy)}
    after = {m.name: f for m, _, f in can_burst([st_top if m == STEERING else m for m in busy])}
    for name in ["Steering command", "Airbag pre-crash", "ESC yaw rate", "Brake command"]:
        print(f"  {name:20s} before {ms(before[name])}   after {ms(after[name])}")
    blocking = can_c
    print(f"\n  Steering at the top still has to wait for:")
    print(f"    a lower-priority frame already on the bus : {ms(blocking)}")
    print(f"    each error + retransmission               : {ms(ERROR_FRAME_BITS / CAN_BITRATE + can_c)}")
    print("  And Airbag, ESC and Brake each got one frame slower: priority only moves\n"
          "  the delay to someone else, it does not remove it.")

    header("Q2 - Why not put every message on FlexRay?")
    print(f"  Static slots per cycle available    : {FR_STATIC_SLOTS}")
    print(f"  Needed for our 5 messages alone     : 5 (plus airbag, ESC, ADAS, ...)")
    print(f"  Use of a 1 ms slot by one frame     : {fr_c / FR_SLOT:.1%}")
    print("  Battery status (100 ms) in a 5 ms slot -> 19 of every 20 slots wasted.")
    print("  Infotainment is event-driven -> its slot would usually be empty.")

    header("Q3 - Fallback when a communication channel is lost")
    print("  Normal       : steering, brake, wheel speed sent on FlexRay A AND B.")
    print(f"  Lose A       : B still delivers them, same latency ({ms(fr_c).strip()}).")
    fb = [replace(STEERING, can_id=0x001), replace(BRAKE, can_id=0x002),
          replace(WHEEL, can_id=0x003), BATTERY]            # infotainment shed
    fb_res = {m.name: f for m, _, f in can_burst(fb + [AIRBAG, ESC] + adas_burst(8))}
    fb_wc = can_worst_case(fb + [AIRBAG, ESC], n_errors=1)
    print("  Lose FlexRay : safety messages move to CAN on reserved top IDs,")
    print("                 infotainment is switched off, car goes to a safe-stop mode.")
    print(f"  {'Message':22s} {'Busy burst':>10} {'Worst case':>11} {'Deadline':>10}")
    for m in fb:
        print(f"  {m.name:22s} {ms(fb_res[m.name])} {ms(fb_wc[m])} {ms(m.deadline)}")

    header("Q4 - Cheapest network that still meets every deadline")
    cheap_wheel = replace(WHEEL, can_id=0x00C)    # high priority, below airbag/ESC
    cheap_can = [cheap_wheel, BATTERY, INFO, AIRBAG, ESC]
    cheap_wc = can_worst_case(cheap_can, n_errors=1)
    cheap_busy = {m.name: f for m, _, f in can_burst(cheap_can + adas_burst(8))}
    print("  FlexRay only for steering + brake; wheel speed moves to CAN at ID 0x00C.")
    print(f"  Wheel speed on CAN: worst case {ms(cheap_wc[cheap_wheel]).strip()}, "
          f"busy burst {ms(cheap_busy[WHEEL.name]).strip()}, deadline 10 ms -> MEETS")
    print("  Fewer FlexRay nodes = fewer FlexRay controllers/transceivers to pay for.")


if __name__ == "__main__":
    main()