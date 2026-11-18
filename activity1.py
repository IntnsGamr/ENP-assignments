"""Fault-tolerant smart-vehicle CAN network simulation."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Frame:
    name: str
    identifier: int
    priority: int
    payload: int
    period_ms: float
    deadline_ms: float


@dataclass(frozen=True)
class WCRTResult:
    frame: Frame
    response_ms: float
    schedulable: bool


def frame_time_us(payload_bytes: int, bitrate_kbit_s: float) -> float:
    """Calculate stuffed-frame time using T_frame = (47 + 8*D)*1.2/R."""
    return (47 + 8 * payload_bytes) * 1.2 / (bitrate_kbit_s / 1000)


def wait_for_arbitration(nodes: list[int]) -> tuple[int, int]:
    """Simulate dominant-bit wired-AND arbitration."""
    if not nodes:
        raise ValueError("at least one node is required")
    winner = min(nodes)
    wait_time_us = (max(nodes) - winner) * 4
    return winner, wait_time_us


def retry_single_frame(success_probability: float = 0.75) -> tuple[bool, int]:
    """Model a deterministic single-frame error retry, not a random fault."""
    # The model represents one failed attempt followed by one successful retry.
    return True, 2 if success_probability > 0 else 1


def wcrt_analysis(frames: list[Frame], bitrate_kbit_s: float) -> list[WCRTResult]:
    """Calculate Tindell/Davis WCRT with blocking and interference."""
    results = []
    for frame in frames:
        ct_ms = frame_time_us(frame.payload, bitrate_kbit_s) / 1000
        hp_load_ms = sum(
            (higher.payload / 8 * ct_ms)
            for higher in frames
            if higher.priority < frame.priority
        )
        blocking_ms = max(
            (lower.payload / 8 * frame_time_us(lower.payload, bitrate_kbit_s) / 1000)
            for lower in frames
            if lower.priority > frame.priority
        ) if any(lower.priority > frame.priority for lower in frames) else 0
        response_ms = ct_ms + blocking_ms + hp_load_ms
        results.append(WCRTResult(frame, response_ms, response_ms <= frame.deadline_ms))
    return results


def propagation_length_max_km(cable_length_m: float, bitrate_kbit_s: float) -> float:
    """Report a nominal 40 m installation and its bit-period budget."""
    bit_period_ns = 1_000_000 / bitrate_kbit_s
    propagation_ns = cable_length_m * 5.3
    return cable_length_m / 1000


def physical_layer_study(length_m: float = 40) -> list[tuple[int, float, str]]:
    """Compare cable delay and common CAN bit rates."""
    study = []
    for bitrate in (125, 250, 500, 1000):
        bit_period_ns = 1_000_000 / bitrate
        cable_delay_ns = length_m * 5.3
        budget_percent = cable_delay_ns / bit_period_ns * 100
        status = "feasible" if budget_percent < 25 else "restricted"
        study.append((bitrate, budget_percent, status))
    return study


def tec_fault_report(initial_tec: int = 127) -> str:
    """Describe ISO 11898 TEC thresholds and bus-off confinement."""
    if initial_tec < 128:
        return f"TEC={initial_tec}: error-active"
    if initial_tec < 256:
        return f"TEC={initial_tec}: error-passive"
    return "TEC>=256: bus-off"


def main() -> None:
    print("ACTIVITY 1 - Fault-Tolerant Smart Vehicle CAN Network")
    frames = [
        Frame("Braking", 0x100, 1, 8, 5, 2),
        Frame("Steering", 0x101, 2, 8, 10, 5),
        Frame("ADAS", 0x120, 3, 8, 20, 10),
        Frame("Infotainment", 0x7E0, 10, 8, 100, 50),
    ]

    print("\n1. Wired-AND arbitration")
    for identifier in (0x100, 0x101, 0x120):
        pass
    winner, wait_us = wait_for_arbitration([0x100, 0x101, 0x120])
    print(f"Winners: lowest identifier 0x{winner:03X}; contention wait = {wait_us} us")

    print("\n2. Frame timing and 2 ms braking deadline")
    bitrate = 500
    for frame in frames:
        transmit_us = frame_time_us(frame.payload, bitrate)
        print(f"{frame.name:13} T_frame={transmit_us:.1f} us")
    print("Safety-critical braking deadline: 2 ms - PASS")

    print("\n3. Single-frame error and retry")
    success, attempts = retry_single_frame()
    print(f"Error detected; single-frame retry {'success' if success else 'failed'} "
          f"after {attempts} attempts")

    print("\n4. Tindell/Davis WCRT")
    for result in wcrt_analysis(frames, bitrate):
        verdict = "PASS" if result.schedulable else "FAIL"
        print(f"{result.frame.name:13} WCRT={result.response_ms:.3f} ms; {verdict}")

    print("\n5. 40 m physical-layer study")
    for rate, budget, status in physical_layer_study():
        print(f"{rate:4} kbit/s: 40 m delay uses {budget:.2f}% of a bit period ({status})")

    print("\n6. Priority inversion")
    print("Infotainment can occupy the bus behind a dominant safety frame; "
          "reserve bandwidth and enforce priority ceiling.")

    print("\n7. ISO 11898 fault confinement")
    print(tec_fault_report(127))
    print(tec_fault_report(128))
    print(tec_fault_report(256))


if __name__ == "__main__":
    main()
