#!/usr/bin/env python3
"""Send real-Go2 kp/kd scales to rl_real_go2's loopback-only UDP interface.

Requires RL_SAR_ENABLE_VLA_BRIDGE=1 when rl_real_go2 starts. Values use
hardware order: FR hip/thigh/calf, FL hip/thigh/calf, RR hip/thigh/calf,
RL hip/thigh/calf. The UDP packet has no acknowledgement or automatic ramp.
"""

import argparse
import math
import socket
import struct


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send 12 Go2 gain scales in hardware order to onboard rl_real_go2."
    )
    parser.add_argument(
        "values", nargs=12, type=float,
        metavar="SCALE",
    )
    parser.add_argument("--port", type=int, default=5560)
    args = parser.parse_args()

    if not 1 <= args.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    if any(not math.isfinite(value) or value < 0.0 or value > 1.0 for value in args.values):
        parser.error("every scale must be finite and in [0.0, 1.0]")

    packet = b"QGNA" + bytes((1,)) + struct.pack("!12f", *args.values)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(packet, ("127.0.0.1", args.port))
    print("Sent gain alpha (UDP, unacknowledged):", " ".join(f"{value:.3f}" for value in args.values))
    print("Confirm 'Applied local gain alpha' in rl_real_go2 output; its bridge must be enabled.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
