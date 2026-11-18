"""Autonomous vehicle hybrid CAN and FlexRay architecture simulation."""

from dataclasses import dataclass


@dataclass(frozen=True)
class BurstResult:
    transport: str
    burst_frames: int
    latency_ms: float


def can_burst_latency(frame_count: int, frame_ms: float) -> float:
    return frame_count * frame_ms


def flexray_burst_latency(frame_count: int, slot_count: int = 4,
                          slot_ms: float = 1.0, cycle_ms: float = 5.0) -> float:
    slot_capacity = slot_count * slot_ms
    if frame_count <= slot_count:
        return slot_capacity / slot_count
    return cycle_ms


def clock_sensitivity(delay_ms: float, skew_ppm: float) -> float:
    return delay_ms * (1 + skew_ppm / 1_000_000)


def failover_latency(primary_ms: float, switchover_ms: float) -> float:
    return primary_ms + switchover_ms


def cost_score(can_fraction: float) -> float:
    """Normalized cost index: CAN controllers are cheaper than FlexRay nodes."""
    return 0.35 + 0.65 * can_fraction


def main() -> None:
    print("ACTIVITY 2 - Autonomous Hybrid CAN + FlexRay Architecture")

    print("\nCAN: 500 kbit/s | FlexRay: 5 Mbit/s")
    print("FlexRay communication cycle: 5 ms; static segment: 4 x 1 ms slots")

    burst_count = 4
    can_frame_ms = 0.2664
    can_latency = can_burst_latency(burst_count, can_frame_ms)
    flex_latency = flexray_burst_latency(burst_count)
    print(f"\nEmergency ADAS burst ({burst_count} frames)")
    print(f"CAN serial latency       = {can_latency:.4f} ms")
    print(f"FlexRay static latency   = {flex_latency:.4f} ms")
    print(f"FlexRay/CAN latency ratio= {flex_latency/can_latency:.3f}")

    print("\nClock-synchronization sensitivity")
    for ppm in (10, 50, 100):
        print(f"{ppm:3} ppm: effective latency = "
              f"{clock_sensitivity(flex_latency, ppm):.5f} ms")

    print("\nDual-channel failover")
    print(f"A/B switchover (detection included) = "
          f"{failover_latency(flex_latency, 0.5):.4f} ms")

    print("\nCost-optimized partitioning")
    for fraction in (0.25, 0.50, 0.75):
        print(f"CAN share={fraction:.0%}: normalized system cost={cost_score(fraction):.2f}")
    print("Recommendation: CAN for body/comfort and FlexRay for ADAS/time-critical control")


if __name__ == "__main__":
    main()
