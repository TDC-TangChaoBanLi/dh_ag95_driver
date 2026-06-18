"""
AG-95 Gripper — Usage Examples

This file demonstrates two transport modes:
  1. SocketCAN (Linux, USB-to-CAN adapter as can0)
  2. PCAN-Basic (PEAK PCAN-USB, recognized as PCAN_USBBUS1)

Assumptions:
  - SocketCAN: can0 is configured and up (sudo ip link set can0 up type can bitrate 500000)
  - PCAN-Basic: PEAK PCAN-USB adapter is connected as PCAN_USBBUS1
  - Grippers at CAN IDs 1 and/or 2 at 500 kbit/s
"""

from dh_ag95 import Ag95Config, TransportType, Ag95Gripper


def test_gripper_socketcan(gripper_id: int):
    """Example using SocketCAN transport (Linux only, can0 interface)."""
    print(f"\n=== SocketCAN — Gripper ID {gripper_id} ===")
    cfg = Ag95Config(transport_type=TransportType.SOCKETCAN)
    cfg.gripper_id = gripper_id
    cfg.socketcan.interface_name = "can0"

    g = Ag95Gripper(cfg)
    try:
        g.connect()
        print("  Connected")

        g.initialize(wait=True)
        print("  Initialized")

        fw = g.get_firmware_version()
        print(f"  Firmware: {fw}")

        state = g.read_state()
        print(f"  Position: {state.position_percent}%")
        print(f"  Force: {state.force_internal_percent}%")
        print(f"  Status: {state.status.name}")

        g.set_force(30)
        g.set_position(50)
        print("  Command: force=30%, position=50%")

    except Exception as e:
        print(f"  ERROR: {e}")
    finally:
        g.disconnect()
        print("  Disconnected")


def test_gripper_pcanbasic(gripper_id: int):
    """Example using PEAK PCAN-Basic transport."""
    print(f"\n=== PCAN-Basic — Gripper ID {gripper_id} ===")
    cfg = Ag95Config(transport_type=TransportType.PCANBASIC)
    cfg.gripper_id = gripper_id
    cfg.pcanbasic.channel = "PCAN_USBBUS1"
    cfg.pcanbasic.bitrate = 500000

    g = Ag95Gripper(cfg)
    try:
        g.connect()
        print("  Connected")

        g.initialize(wait=True)
        print("  Initialized")

        fw = g.get_firmware_version()
        print(f"  Firmware: {fw}")

        state = g.read_state()
        print(f"  Position: {state.position_percent}%")
        print(f"  Force: {state.force_internal_percent}%")
        print(f"  Status: {state.status.name}")

        g.set_force(30)
        g.set_position(50)
        print("  Command: force=30%, position=50%")

    except Exception as e:
        print(f"  ERROR: {e}")
    finally:
        g.disconnect()
        print("  Disconnected")


if __name__ == "__main__":
    import sys

    # Select transport mode from command line: "socketcan" or "pcanbasic" (default)
    mode = sys.argv[1] if len(sys.argv) > 1 else "socketcan"

    if mode == "socketcan":
        for gid in [1, 2]:
            test_gripper_socketcan(gid)
    elif mode == "pcanbasic":
        for gid in [1, 2]:
            test_gripper_pcanbasic(gid)
    else:
        print(f"Unknown mode '{mode}'. Use 'socketcan' or 'pcanbasic'.")
    print("\nDone.")