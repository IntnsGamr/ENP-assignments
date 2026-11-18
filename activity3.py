"""SJA1000 external CAN controller selection and acceptance filtering demo."""


CAN_ACR = 0x24
CAN_AMR = 0x03
TARGET_ID = 0x123


def standard_id_acr_amr(identifier: int) -> tuple[int, int]:
    """Return PeliCAN-style 11-bit standard-ID code and mask fields."""
    return (identifier >> 3) & 0xFF, 0xFF ^ (identifier & 0x07)


def acceptance_match(identifier: int, acr: int, amr: int) -> bool:
    """Perform SJA1000 ACR/AMR comparison using don't-care mask bits."""
    code_field = (identifier >> 3) & 0xFF
    low_field = identifier & 0x07
    code_match = ((code_field ^ acr) & (0xFF ^ amr)) == 0
    mask_low = amr & 0x07
    low_match = ((low_field ^ 0) & (0x07 ^ mask_low)) == 0
    return code_match and low_match


def verify_123(acr: int = CAN_ACR, amr: int = CAN_AMR,
               identifier: int = TARGET_ID) -> bool:
    high_match = ((identifier >> 3) & 0xFF) == acr
    mask_low = amr & 0x07
    low_match = ((identifier & 0x07) & (0x07 ^ mask_low)) == 0
    return bool(high_match and low_match)


def main() -> None:
    print("ACTIVITY 3 - SJA1000 External CAN Controller Interface")
    print("\nHardware interface:")
    print("MCU -> SPI/Parallel -> SJA1000 -> CAN Transceiver -> CAN Bus")

    print("\nController selection")
    print("32 identifiers: Basic CAN is sufficient for one receive buffer per use case.")
    print("Full CAN/PeliCAN is recommended for 32 independently configured filters.")

    print("\nAcceptance Code/Mask verification")
    print(f"Target ID = 0x{TARGET_ID:03X}")
    print(f"ACR       = 0x{CAN_ACR:02X}")
    print(f"AMR       = 0x{CAN_AMR:02X}")
    acr_field = (TARGET_ID >> 3) & 0xFF
    low_field = TARGET_ID & 0x07
    print(f"Standard-ID code field: 0x{acr_field:02X}")
    print(f"Low-order don't-care field: 0b{low_field:03b}")
    print(f"Verification = {'PASS' if verify_123() else 'FAIL'}")
    print("Result: ID 0x123 is accepted by the configured SJA1000 filter")


if __name__ == "__main__":
    main()
