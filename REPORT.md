# Automotive In-Vehicle Networking: Fault-Tolerant CAN, Hybrid FlexRay & SJA1000 Architecture

## Abstract

This report presents three complementary simulations for an automotive in-vehicle
network. The first evaluates fault-tolerant CAN behavior, including arbitration,
timing, error recovery, and schedulability. The second compares a hybrid CAN/FlexRay
architecture for autonomous vehicle traffic. The third evaluates SJA1000 controller
selection and acceptance filtering. The analysis emphasizes deterministic deadlines,
fault containment, physical-layer constraints, and cost-effective partitioning.

## Author and Submission Details

| Field | Details |
|---|---|
| Name | Namith Anilkumar Mathod |
| USN | 1MS24ET030 (Roll No. 30, Batch B2) |
| Department | Electronics and Telecommunication Engineering, Ramaiah Institute of Technology, Bengaluru |

# 1. Activity 1: Fault-Tolerant CAN Network

## 1.1 Methodology

An 11-bit identifier is modeled as the CAN arbitration priority: the lower binary
identifier wins on the dominant wired-AND bus. Braking is assigned priority 1,
steering priority 2, ADAS priority 3, and infotainment priority 10. A frame payload
of eight bytes is used as the common worst-case workload.

The stuffed-frame transmission time is modeled by:

\[
T_{frame}=\frac{(47+8D)\times1.2}{R}
\]

where \(D\) is payload bytes and \(R\) is the CAN data rate in kbit/s. The factor 1.2
is a conservative 20-percent stuffing allowance.

For one non-cumulative fault, error recovery is modeled as one detected failed
attempt plus one retransmission. ISO 11898 TEC states are:

| TEC | State | Behavior |
|---:|---|---|
| 0-127 | Error-active | Normal participation; active error frames |
| 128-255 | Error-passive | No active error frame generation |
| >=256 | Bus-off | Node is disconnected from the bus |

## 1.2 Worst-Case Response Time

The Tindell/Davis response-time formulation is:

\[
R_i=C_i+B_i+\sum_{j\in hp(i)}
\left\lceil\frac{R_i}{T_j}\right\rceil C_j
\]

where \(C_i\) is frame transmission time, \(B_i\) is blocking from a lower-priority
busy frame, \(T_j\) is the higher-priority frame period, and \(hp(i)\) is the set of
higher-priority frames. The braking frame must satisfy \(R_{\text{braking}}\leq2\) ms.

At 500 kbit/s, an eight-byte frame is approximately 266.4 microseconds. A simplified
high-priority and blocking calculation yields sub-2 ms WCRT for braking in the
demonstration workload.

## 1.3 Physical-Layer Study

For a 40 m cable, the nominal propagation delay is approximately \(40\times5.3=212\) ns.
At 1 Mbit/s the bit period is 1 microsecond, leaving a small but constrained margin
for transceiver and node timing. The propagation-delay and phase-segment budget is:

\[
L_{max}= \frac{(T_{bit}/2-T_{node}-T_{transceiver})v}{2}
\]

with typical cabling velocity \(v\). Practical CAN transceiver propagation and oscillator
tolerance should be included; the 40 m distance is not ISO 11898's maximum definition
and is feasible only if the complete network delay budget is verified.

| Cable / rate | Nominal cable delay | 1-bit-period fraction |
|---|---:|---:|
| 40 m, 125 kbit/s | 212 ns | 2.65% |
| 40 m, 250 kbit/s | 212 ns | 5.30% |
| 40 m, 500 kbit/s | 212 ns | 10.60% |
| 40 m, 1 Mbit/s | 212 ns | 21.20% |

## 1.4 Fault Tolerance and Priority Inversion

Infotainment traffic must not hold up a safety-critical frame. In addition to identifier
priority and bandwidth reservation, gateways should use traffic shaping and a priority
ceiling to prevent software or gateway queue inversion. A failed frame is retried once,
followed by TEC tracking and node isolation if the fault persists.

# 2. Activity 2: Hybrid CAN and FlexRay

## 2.1 Methodology

CAN is modeled at 500 kbit/s and FlexRay at 5 Mbit/s. The FlexRay communication cycle
is 5 ms and includes four 1 ms static slots. CAN is suitable for event-triggered,
lower-cost body and infotainment traffic; FlexRay provides deterministic time-triggered
delivery for safety and autonomous control.

For a burst of four frames, CAN serial latency is approximately:

\[
L_{CAN}=N \times T_{frame}
\]

The demonstrated eight-byte CAN frame time is 266.4 us, giving approximately 1.0656 ms
for four frames. The static FlexRay burst can occupy a 1 ms slot set within the 5 ms
cycle, making cycle alignment an important upper-bound contribution.

## 2.2 Numerical Results

| Metric | CAN 500 kbit/s | FlexRay 5 Mbit/s |
|---|---:|---:|
| Cycle/serial model | Contention-based | 5 ms communication cycle |
| Static capacity | Shared full bus | Four 1 ms slots |
| Four-frame burst | ~1.066 ms | ~1.000 ms slot model |
| Latency ratio (FlexRay/CAN) | Reference 1.000 | ~0.939 |

Clock synchronization sensitivity is modeled linearly:

\[
L_{effective}=L_{nominal}(1+p/10^6)
\]

Dual-channel failover adds a 0.5 ms detection and switch margin to the nominal channel
latency. The failover result is approximately 1.5 ms for this model.

## 2.3 Cost-Optimized Partitioning

CAN remains preferred for comfort and body networks because it requires lower hardware
and software cost. FlexRay is allocated to domains whose deadlines or determinism justify
the added nodes, gateways, and development effort. A practical recommendation is CAN
for infotainment and body traffic, FlexRay for ADAS and time-critical control, and isolated
CAN-to-FlexRay gateways with bounded queues.

# 3. Activity 3: SJA1000 External CAN Controller

## 3.1 Interface and Controller Selection

The external hardware chain is:

\[
\text{MCU}\rightarrow\text{SPI/Parallel}\rightarrow\text{SJA1000}
\rightarrow\text{CAN Transceiver}\rightarrow\text{CAN Bus}
\]

Basic CAN is economical when messages are received to one or a few single receive buffers.
For 32 independent identifiers, Full CAN/PeliCAN operation offers separate acceptance
register configuration and is the clearer architectural selection.

## 3.2 Acceptance Code/Mask Verification

For standard 11-bit identifier \(0x123\), the high code field is:

\[
ACR=(0x123>>3)\&0xFF=0x24
\]

With \(AMR=0x03\), the low identifier field \(0x3\) is subject to the mask. The
demonstration defines the three low-order bits as the masked field and verifies:

\[
((0x123>>3)\oplus0x24)=0
\]

Therefore ID \(0x123\) is accepted by the configured filter. A received list of test
identifiers should include neighboring values to verify don't-care behavior against the
actual SJA1000 datasheet register width and PeliCAN mode.

# 4. Cross-Activity Engineering Recommendations

| Recommendation | Rationale |
|---|---|
| Reserve CAN bandwidth and assign braking a high identifier priority | Improves deadline margin and reduces priority inversion |
| Configure TEC state monitoring and controlled bus-off recovery | Prevents a persistently faulty node from corrupting network availability |
| Use FlexRay for deterministic ADAS/control and CAN for cost-sensitive domains | Separates latency-critical traffic from broader body-network traffic |
| Implement dual-channel FlexRay failover and watchdog supervision | Provides channel redundancy and bounded switchover |
| Use PeliCAN for 32 independently filtered identifiers | Avoids software filtering bottlenecks and simplifies MCU receive logic |
| Verify the complete physical-layer delay budget at 40 m | The cable-length study alone does not replace timing/transceiver analysis |

## Final Conclusion

A fault-tolerant smart-vehicle network should retain CAN for high-volume and cost-sensitive
traffic, but reserve it through deterministic scheduling and TEC-aware fault management.
FlexRay should carry the time-critical autonomous-control domain when its 5 ms cycle and
channel redundancy can be justified. The SJA1000 PeliCAN configuration provides the
appropriate hardware filtering capability for 32 identifiers.
