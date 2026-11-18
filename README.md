# Automotive In-Vehicle Networking: Fault-Tolerant CAN, Hybrid FlexRay & SJA1000 Architecture

## Author

| Field | Details |
|---|---|
| Name | Namith Anilkumar Mathod |
| USN | 1MS24ET030 (Roll No. 30, Batch B2) |
| Department | Electronics and Telecommunication Engineering |
| Institution | Ramaiah Institute of Technology, Bengaluru |

## Repository Structure

| File | Description |
|---|---|
| `activity1.py` | Fault-tolerant smart vehicle CAN network simulation: 11-bit wired-AND arbitration, burst waiting times, 2 ms safety-critical deadline verification, single-frame error retry, Tindell/Davis Worst-Case Response Time analysis, cable-length versus bit-rate physical-layer study at 40 m, priority inversion with infotainment traffic, and ISO 11898 TEC/bus-off fault confinement. |
| `activity2.py` | Autonomous vehicle hybrid CAN + FlexRay architecture: 500 kbit/s CAN versus 5 Mbit/s time-triggered FlexRay with a 5 ms communication cycle and four 1 ms static slots, emergency ADAS burst latency comparison, clock-synchronization sensitivity, dual-channel A/B failover, and cost-optimized partitioning. |
| `activity3.py` | SJA1000 external CAN controller hardware interface selection (`MCU -> SPI/Parallel -> SJA1000 -> CAN Transceiver -> CAN Bus`), Basic CAN versus Full CAN (PeliCAN mode) selection for 32 identifiers, and bitwise Acceptance Code/Mask Register (`ACR=0x24`, `AMR=0x03`) verification for ID `0x123`. |
| `demo_output.mp4` | Screen recording demonstrating execution and console outputs of all three scripts. |
| `REPORT.md` | Formal technical submission report with methodology, equations, findings, trade-offs, and recommendations. |

## How to Run

The projects use only the Python 3 standard library.

```bash
python3 activity1.py
python3 activity2.py
python3 activity3.py
```