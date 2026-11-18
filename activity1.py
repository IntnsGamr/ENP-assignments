#!/usr/bin/env python3
"""
Activity 1 - Fault-tolerant CAN network for a smart vehicle
===========================================================

Covers the six required tasks:
  1. Bit-wise CAN arbitration and ordering of messages by identifier
  2. Transmission duration and cumulative waiting time per message
  3. Deadline check (2 ms) for the safety-critical messages
  4. A transmission error followed by an automatic retry
  5. Whether raising the bit rate is always the best improvement
  6. A recommended communication strategy

It also includes the extensions asked about in the questions:
  - Q1: ABS losing priority to a non-critical infotainment stream
  - Q3: random bit errors, bus load over time, TEC counters, bus-off
  - Q4: worst-case response-time analysis (not just the t = 0 burst)

Frame time uses the teaching approximation from the activity sheet:
      T_frame = (47 + 8*D) * 1.2 / R
Only the Python standard library is used.  Run:  python can_sim.py
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

# --------------------------------------------------------------------------
# Network parameters (from the problem statement)
# --------------------------------------------------------------------------
BITRATE = 500_000          # nominal bit rate, bit/s
CABLE_LENGTH = 40          # m
DEADLINE_CRIT = 2e-3       # s, deadline for safety-critical messages

FIXED_OVERHEAD_BITS = 47   # assumed fixed frame overhead
STUFF_FACTOR = 1.2         # allowance for bit stuffing etc.

# --------------------------------------------------------------------------
# Error handling model (ISO 11898 fault confinement, simplified)
# --------------------------------------------------------------------------
# Error frame = error flag (6 bits, up to 12 when flags superpose)
#             + error delimiter (8) + intermission (3). Worst case used.
ERROR_FRAME_BITS = 12 + 8 + 3
TEC_ERROR_INC = 8              # transmit error counter +8 per tx error
TEC_SUCCESS_DEC = 1            # -1 per successful transmission
ERROR_PASSIVE_LIMIT = 128      # TEC >= 128 -> error-passive
BUS_OFF_LIMIT = 256            # TEC >= 256 -> bus-off
BUS_OFF_RECOVERY_BITS = 128 * 11   # 128 x 11 recessive bits before rejoining

# --------------------------------------------------------------------------
# Physical-layer assumptions for the bit-rate study (typical values)
# --------------------------------------------------------------------------
PROP_DELAY_PER_M = 5e-9    # s/m, signal speed on twisted pair (~0.66 c)
NODE_LOOP_DELAY = 150e-9   # s, one-way controller + transceiver delay
PROP_SEG_FRACTION = 0.70   # share of a bit time usable for propagation
LENGTH_MARGIN_REQ = 0.25   # want >= 25 % spare cable-length budget


# --------------------------------------------------------------------------
# Data model
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class Message:
    ecu: str
    name: str
    can_id: int        # 11-bit standard identifier
    dlc: int           # data bytes
    period: float      # s
    critical: bool

    @property
    def deadline(self) -> float:
        # Critical frames: 2 ms. Others: implicit deadline = period.
        return DEADLINE_CRIT if self.critical else self.period

    @property
    def label(self) -> str:
        return f"{self.ecu} (0x{self.can_id:03X})"


MESSAGES = [
    Message("ABS",       "Emergency braking", 0x080, 8, 10e-3, True),
    Message("Airbag",    "Crash status",      0x040, 8, 20e-3, True),
    Message("Engine",    "Engine speed",      0x180, 8, 10e-3, False),
    Message("Dashboard", "Vehicle speed",     0x300, 8, 50e-3, False),
]


def us(seconds: float) -> str:
    return f"{seconds * 1e6:8.1f} us"


def frame_time(dlc: int, bitrate: float) -> float:
    """Teaching approximation of one frame's transmission time (s)."""
    return (FIXED_OVERHEAD_BITS + 8 * dlc) * STUFF_FACTOR / bitrate


def error_overhead(bitrate: float) -> float:
    return ERROR_FRAME_BITS / bitrate


def header(title: str) -> None:
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)


# --------------------------------------------------------------------------
# Task 1 - bit-wise arbitration
# --------------------------------------------------------------------------
def arbitrate(contenders: list[Message], trace: bool = False) -> Message:
    """
    Simulate CAN arbitration on the 11-bit identifier, MSB first.
    The bus is wired-AND: a dominant 0 overwrites a recessive 1. A node that
    sends recessive but reads dominant has lost and backs off.
    """
    alive = list(contenders)
    if len({m.can_id for m in alive}) != len(alive):
        raise ValueError("Two nodes with the same CAN ID - not allowed on CAN")
    for bit in range(10, -1, -1):
        bus = min((m.can_id >> bit) & 1 for m in alive)
        losers = [m for m in alive if (m.can_id >> bit) & 1 != bus]
        if losers and trace:
            names = ", ".join(m.ecu for m in losers)
            print(f"    ID bit {bit:2d}: bus={'dominant(0)' if bus == 0 else 'recessive(1)'}"
                  f"  -> {names} back off")
        alive = [m for m in alive if (m.can_id >> bit) & 1 == bus]
        if len(alive) == 1:
            break
    return alive[0]


def arbitration_order(msgs: list[Message], trace: bool = False) -> list[Message]:
    remaining, order, rnd = list(msgs), [], 1
    while remaining:
        if trace:
            print(f"  Round {rnd}: contenders = {[m.ecu for m in remaining]}")
        winner = arbitrate(remaining, trace)
        if trace:
            print(f"    winner: {winner.label}")
        order.append(winner)
        remaining.remove(winner)
        rnd += 1
    return order


# --------------------------------------------------------------------------
# Tasks 2-4 - burst simulation (all frames ready at t = 0, optional errors)
# --------------------------------------------------------------------------
@dataclass
class TxResult:
    msg: Message
    first_ready: float
    start: float       # start of the successful transmission
    finish: float      # end of the successful transmission (= response time)
    attempts: int


def simulate_burst(msgs, bitrate, errors=None, log=False):
    """
    All messages ready at t = 0. `errors` maps ECU name -> number of its
    transmission attempts to corrupt. A corrupted frame is assumed to be
    detected at its end (worst case, e.g. CRC/ACK error), followed by an
    error frame; the sender then re-enters arbitration automatically.
    """
    errors = dict(errors or {})
    pending = list(msgs)
    attempts = {m: 0 for m in msgs}
    t, results = 0.0, []
    while pending:
        winner = arbitrate(pending)
        c = frame_time(winner.dlc, bitrate)
        attempts[winner] += 1
        if errors.get(winner.ecu, 0) > 0:
            errors[winner.ecu] -= 1
            if log:
                print(f"  t={us(t)}  {winner.label:18s} transmits ... ERROR detected "
                      f"-> error frame ({ERROR_FRAME_BITS} bits), retry")
            t += c + error_overhead(bitrate)
            continue
        if log:
            print(f"  t={us(t)}  {winner.label:18s} transmits OK "
                  f"(attempt {attempts[winner]}), done at {us(t + c).strip()}")
        results.append(TxResult(winner, 0.0, t, t + c, attempts[winner]))
        t += c
        pending.remove(winner)
    return results


def print_burst_table(results, bitrate):
    print(f"  {'#':>2} {'Message':18s} {'Tx time':>11} {'Wait':>11} "
          f"{'Finish':>11} {'Deadline':>10} {'Tries':>5}  Status")
    total_wait = 0.0
    for i, r in enumerate(results, 1):
        tx = frame_time(r.msg.dlc, bitrate)
        wait = r.start - r.first_ready
        total_wait += wait
        ok = r.finish <= r.msg.deadline
        status = ("MEETS" if ok else "MISSES") + (" (critical)" if r.msg.critical else "")
        print(f"  {i:>2} {r.msg.label:18s} {us(tx)} {us(wait)} {us(r.finish)} "
              f"{r.msg.deadline * 1e3:7.1f} ms {r.attempts:>5}  {status}")
    print(f"  Sum of waiting times: {us(total_wait).strip()}; "
          f"bus busy until {us(results[-1].finish).strip()}")


# --------------------------------------------------------------------------
# Worst-case response-time analysis (Tindell / Davis style, simplified)
# --------------------------------------------------------------------------
def response_time_analysis(msgs, bitrate, n_errors=1):
    """
    R = B + I + E + C, where
      B = blocking by one lower-priority frame already on the bus
          (CAN is non-preemptive),
      I = interference from higher-priority frames released meanwhile,
      E = recovery cost of n_errors (error frame + longest retransmission),
      C = own frame time.
    Jitter and the own-previous-instance push-through effect are ignored.
    Returns {msg: R or math.inf if it diverges}.
    """
    tbit = 1 / bitrate
    ordered = sorted(msgs, key=lambda m: m.can_id)
    out = {}
    for i, m in enumerate(ordered):
        c = frame_time(m.dlc, bitrate)
        hp, lp = ordered[:i], ordered[i + 1:]
        blocking = max((frame_time(x.dlc, bitrate) for x in lp), default=0.0)
        err = n_errors * (ERROR_FRAME_BITS * tbit
                          + max(frame_time(x.dlc, bitrate) for x in hp + [m]))
        w = blocking + err
        while True:
            w_new = blocking + err + sum(
                math.ceil((w + tbit) / x.period) * frame_time(x.dlc, bitrate) for x in hp)
            if abs(w_new - w) < 1e-12:
                out[m] = w + c
                break
            if w_new > 100 * m.period:     # not converging -> overloaded
                out[m] = math.inf
                break
            w = w_new
    return out


def bus_load(msgs, bitrate):
    return sum(frame_time(m.dlc, bitrate) / m.period for m in msgs)


# --------------------------------------------------------------------------
# Task 5 - physical-layer check for the bit-rate study
# --------------------------------------------------------------------------
def max_bus_length(bitrate):
    """
    All nodes must see the same bit before the sample point during
    arbitration, so the round trip across the bus must fit in the part of
    the bit time reserved for propagation:
        2 * (L * t_prop + t_node) <= PROP_SEG_FRACTION * t_bit
    """
    budget = PROP_SEG_FRACTION / bitrate - 2 * NODE_LOOP_DELAY
    return max(0.0, budget / (2 * PROP_DELAY_PER_M))


def bitrate_study(msgs, rates, length):
    print(f"  Assumptions: {PROP_DELAY_PER_M * 1e9:.0f} ns/m cable delay, "
          f"{NODE_LOOP_DELAY * 1e9:.0f} ns node delay, "
          f"{PROP_SEG_FRACTION:.0%} of bit time for propagation, 1 error in the window")
    print(f"  {'Rate':>9} {'Frame':>11} {'Load':>6} {'WC crit resp':>13} "
          f"{'Lmax':>7} {'Len margin':>10}  Verdict")
    rows = []
    for r in rates:
        rta = response_time_analysis(msgs, r, n_errors=1)
        crit_wc = max(rta[m] for m in msgs if m.critical)
        timing_ok = all(rta[m] <= m.deadline for m in msgs)
        lmax = max_bus_length(r)
        margin = lmax / length - 1
        phys_ok = margin >= LENGTH_MARGIN_REQ
        if timing_ok and phys_ok:
            verdict = "OK"
        elif timing_ok:
            verdict = "timing OK, but no physical margin" if margin >= 0 else "timing OK, cable too long"
        else:
            verdict = "deadline missed (worst case)"
        rows.append((r, timing_ok, phys_ok, margin))
        print(f"  {r / 1e3:6.0f} k {us(frame_time(8, r))} {bus_load(msgs, r):6.1%} "
              f"{us(crit_wc)}  {lmax:5.0f} m {margin:+9.0%}  {verdict}")
    return rows


# --------------------------------------------------------------------------
# Q3 extension - time-line simulation with bus load, random errors,
# TEC counters, error-passive and bus-off
# --------------------------------------------------------------------------
def simulate_timeline(msgs, bitrate, duration, ber=0.0, faulty_ecu=None,
                      fault_window=(0.0, math.inf), seed=1, auto_recover=True,
                      verbose_states=True):
    """
    Discrete-event simulation of periodic traffic.
      ber          random bit-error rate on the bus (each frame fails with
                   probability 1-(1-ber)^bits)
      faulty_ecu   ECU whose transmissions ALL fail inside fault_window
                   (e.g. a broken transceiver)
    Each ECU has one transmit buffer per message; a new release while the old
    frame is still waiting overwrites it (newest data wins).
    """
    rng = random.Random(seed)
    tbit = 1 / bitrate
    next_rel = {m: 0.0 for m in msgs}
    pending: dict[Message, float] = {}
    ecus = {m.ecu for m in msgs}
    tec = {e: 0 for e in ecus}
    state = {e: "error-active" for e in ecus}
    off_until = {e: math.inf for e in ecus}
    stats = {m: dict(sent=0, max_r=0.0, miss=0, errors=0, overwritten=0, dropped=0)
             for m in msgs}
    busy, t, events = 0.0, 0.0, []

    def set_state(ecu, new):
        if state[ecu] != new:
            events.append((t, ecu, state[ecu], new, tec[ecu]))
            state[ecu] = new

    while t < duration:
        # Bus-off recovery
        for e in ecus:
            if state[e] == "bus-off" and auto_recover and t >= off_until[e]:
                tec[e] = 0
                off_until[e] = math.inf
                set_state(e, "error-active")
        # Periodic releases
        for m in msgs:
            while next_rel[m] <= t:
                if state[m.ecu] == "bus-off":
                    stats[m]["dropped"] += 1
                else:
                    if m in pending:
                        stats[m]["overwritten"] += 1
                    pending[m] = next_rel[m]
                next_rel[m] += m.period
        ready = [m for m in pending if state[m.ecu] != "bus-off"]
        if not ready:      # bus idle: jump to the next event
            t = min(list(next_rel.values()) + [v for v in off_until.values() if v > t])
            continue

        winner = arbitrate(ready)
        c = frame_time(winner.dlc, bitrate)
        in_fault = (winner.ecu == faulty_ecu and fault_window[0] <= t < fault_window[1])
        p_frame_err = 1 - (1 - ber) ** (c * bitrate) if ber else 0.0
        if in_fault or rng.random() < p_frame_err:
            stats[winner]["errors"] += 1
            dt = c + ERROR_FRAME_BITS * tbit
            busy += dt
            t += dt
            tec[winner.ecu] += TEC_ERROR_INC
            if tec[winner.ecu] >= BUS_OFF_LIMIT:
                set_state(winner.ecu, "bus-off")
                off_until[winner.ecu] = t + BUS_OFF_RECOVERY_BITS * tbit
                for m in [m for m in pending if m.ecu == winner.ecu]:
                    del pending[m]
                    stats[m]["dropped"] += 1
            elif tec[winner.ecu] >= ERROR_PASSIVE_LIMIT:
                set_state(winner.ecu, "error-passive")
            continue

        tec[winner.ecu] = max(0, tec[winner.ecu] - TEC_SUCCESS_DEC)
        if state[winner.ecu] == "error-passive" and tec[winner.ecu] < ERROR_PASSIVE_LIMIT:
            set_state(winner.ecu, "error-active")
        resp = t + c - pending.pop(winner)
        s = stats[winner]
        s["sent"] += 1
        s["max_r"] = max(s["max_r"], resp)
        if resp > winner.deadline:
            s["miss"] += 1
        busy += c
        t += c

    print(f"  Measured bus utilisation: {busy / duration:.1%}")
    print(f"  {'Message':18s} {'Sent':>6} {'Errors':>7} {'Max resp':>11} "
          f"{'Deadline':>9} {'Misses':>7} {'Dropped':>8} {'Overwr.':>8}")
    for m in sorted(msgs, key=lambda x: x.can_id):
        s = stats[m]
        print(f"  {m.label:18s} {s['sent']:>6} {s['errors']:>7} {us(s['max_r'])} "
              f"{m.deadline * 1e3:6.1f} ms {s['miss']:>7} {s['dropped']:>8} {s['overwritten']:>8}")
    if verbose_states and events:
        print("  Fault-confinement state changes (first 8):")
        for (te, ecu, old, new, c) in events[:8]:
            print(f"    t={te * 1e3:9.3f} ms  {ecu:9s} {old:>13s} -> {new:<13s} (TEC={c})")
        if len(events) > 8:
            print(f"    ... {len(events) - 8} more")
    return stats


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def main():
    R = BITRATE
    header("TASK 1 - CAN arbitration (all four frames ready at t = 0)")
    order = arbitration_order(MESSAGES, trace=True)
    print("\n  Transmission order: " + "  ->  ".join(m.label for m in order))

    header(f"TASKS 2 & 3 - Timing at {R / 1e3:.0f} kbit/s, error-free")
    print(f"  T_frame = (47 + 8*8) * 1.2 / {R} = {us(frame_time(8, R)).strip()} per 8-byte frame")
    base = simulate_burst(MESSAGES, R)
    print_burst_table(base, R)
    crit_ok = all(r.finish <= DEADLINE_CRIT for r in base if r.msg.critical)
    print(f"\n  Safety-critical messages within 2 ms: {'YES' if crit_ok else 'NO'}")

    header("TASK 4 - ABS frame corrupted once, automatic retransmission")
    err = simulate_burst(MESSAGES, R, errors={"ABS": 1}, log=True)
    print()
    print_burst_table(err, R)
    abs_base = next(r.finish for r in base if r.msg.ecu == "ABS")
    abs_err = next(r.finish for r in err if r.msg.ecu == "ABS")
    print(f"\n  ABS delayed by {us(abs_err - abs_base).strip()} "
          f"(failed frame + {ERROR_FRAME_BITS}-bit error frame). Because ABS keeps "
          "its ID, it\n  wins re-arbitration against Engine/Dashboard and still "
          "meets the 2 ms deadline.")

    header("WORST-CASE RESPONSE TIME (any release phasing, 1 error allowed)")
    rta = response_time_analysis(MESSAGES, R, n_errors=1)
    for m in sorted(MESSAGES, key=lambda x: x.can_id):
        ok = "MEETS" if rta[m] <= m.deadline else "MISSES"
        print(f"  {m.label:18s} R_worst = {us(rta[m])}  deadline {m.deadline * 1e3:5.1f} ms  {ok}")
    print(f"  Average bus load: {bus_load(MESSAGES, R):.1%}")

    header("TASK 5 - Is a higher bit rate always better?")
    rows = bitrate_study(MESSAGES, [125_000, 250_000, 500_000, 1_000_000], CABLE_LENGTH)
    print(f"\n  At {CABLE_LENGTH} m, 1 Mbit/s is at the physical limit (no margin for stubs,\n"
          "  connectors, temperature or transceiver tolerances) while 500 kbit/s\n"
          "  already meets every deadline at < 10 % load. Faster is NOT automatically better.")

    header("Q1 - ABS outranked by an 8-frame infotainment burst (IDs 0x020-0x027)")
    info = [Message(f"Info#{k}", "Media/display", 0x020 + k, 8, 100e-3, False)
            for k in range(8)]
    q1 = simulate_burst(MESSAGES + info, R)
    for r in q1:
        if r.msg.critical:
            ok = "MEETS" if r.finish <= DEADLINE_CRIT else "MISSES"
            print(f"  {r.msg.label:18s} finishes at {us(r.finish)}  -> {ok} 2 ms deadline")

    header("Q3a - 10 s of periodic traffic on a noisy bus (BER = 1e-4)")
    simulate_timeline(MESSAGES, R, duration=10.0, ber=1e-4, seed=7)

    header("Q3b - Engine transceiver fails between t = 1 s and t = 2 s")
    simulate_timeline(MESSAGES, R, duration=3.0, faulty_ecu="Engine",
                      fault_window=(1.0, 2.0), seed=7)
    print("  The faulty node is driven to bus-off after 32 failed attempts, so it\n"
          "  cannot keep jamming the bus; ABS and Airbag keep meeting their deadlines.")

    header("TASK 6 - Recommendation (derived from the results above)")
    chosen = next((r for r, t_ok, p_ok, _ in rows if t_ok and p_ok), None)
    print(f"  * Bit rate: {chosen / 1e3:.0f} kbit/s - the slowest rate that meets all worst-case\n"
          "    deadlines (with one error) AND leaves >= 25 % cable-length margin.")
    print("  * ID plan: reserve the lowest IDs for safety-critical frames\n"
          "    (Airbag 0x040, ABS 0x080); non-critical and diagnostic traffic gets\n"
          "    higher IDs. Never let infotainment use IDs below the safety range.")
    print("  * Put infotainment/comfort traffic on a separate bus behind a gateway\n"
          "    that rate-limits what it forwards onto the safety bus.")
    print("  * Keep safety-bus load low (< ~30-40 %) and verify with worst-case\n"
          "    response-time analysis including an error model - not just one test.")
    print("  * Use CAN fault confinement (TEC/REC, bus-off), heartbeat/timeout\n"
          "    monitoring of critical frames, and a bus guardian against a\n"
          "    'babbling idiot'. For higher bandwidth prefer CAN FD (fast data\n"
          "    phase, arbitration still at a robust rate) over raising the nominal rate.")


if __name__ == "__main__":
    main()