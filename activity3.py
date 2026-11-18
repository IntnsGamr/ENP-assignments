def check_can_architecture(mc_has_can, num_identifiers, req_id, data_len, acr, amr):
    print("--- CAN Hardware Architecture Selection ---")

    # 1. Hardware Architecture Selection
    if not mc_has_can:
        print(
            "Hardware Architecture: Microcontroller -> SPI/Parallel Bus -> SJA1000 CAN Controller -> CAN Transceiver -> CAN Bus")
    else:
        print("Hardware Architecture: Microcontroller (Internal CAN) -> CAN Transceiver -> CAN Bus")

    # 2. Basic vs Full CAN Selection
    # Threshold is typically set around 10-15 IDs. Higher amounts require Full CAN to prevent CPU overload.
    if num_identifiers > 15:
        print(f"Controller Mode: FULL CAN (SJA1000 PeliCAN mode) is suitable.")
        print(
            f"Reason: System handles {num_identifiers} identifiers. Full CAN provides a larger 64-byte FIFO and offloads CPU overhead.")
    else:
        print(f"Controller Mode: BASIC CAN is suitable.")
        print(f"Reason: Low number of identifiers ({num_identifiers}) can be managed by a simple FIFO buffer.")

    # 3. Acceptance Filter Verification
    print("\n--- Acceptance Filter Verification ---")
    print(f"Required CAN ID: {hex(req_id)}")
    print(f"ACR (Acceptance Code): {hex(acr)}")
    print(f"AMR (Acceptance Mask): {hex(amr)}")

    # In SJA1000 standard frame filtering, the top 8 bits of the 11-bit ID are compared
    id_top_8_bits = req_id >> 3

    # Bitwise logic:
    # AMR bits set to '1' are "don't care". Bits set to '0' must match.
    # We use bitwise AND with the inverse of AMR (~amr & 0xFF) to isolate the bits that MUST match.
    relevant_mask = ~amr & 0xFF

    id_relevant_bits = id_top_8_bits & relevant_mask
    acr_relevant_bits = acr & relevant_mask

    print(f"ID Top 8 Bits: {hex(id_top_8_bits)}")

    if id_relevant_bits == acr_relevant_bits:
        print("Verdict: MATCH. The CAN hardware will ACCEPT message ID 0x123.")
    else:
        print("Verdict: NO MATCH. The CAN hardware will REJECT message ID 0x123.")

    # 4. Data Length Check
    if data_len <= 8:
        print(f"Data Length: {data_len} bytes is valid (Standard CAN supports up to 8 bytes).")
    else:
        print(f"Data Length: {data_len} bytes is invalid for standard CAN.")


# Input parameters from the problem statement
check_can_architecture(
    mc_has_can=False,
    num_identifiers=32,
    req_id=0x123,
    data_len=8,
    acr=0x24,
    amr=0x03
)