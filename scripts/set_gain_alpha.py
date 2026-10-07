#!/usr/bin/env python3
"""Set Go2 kp/kd scales on Gazebo or the real robot.

The default sends to rl_real_go2's loopback-only UDP interface and requires
RL_SAR_ENABLE_VLA_BRIDGE=1 when rl_real_go2 starts. --gazebo publishes to
rl_sim's ROS2 gain topic instead. Values use hardware order:
FR hip/thigh/calf, FL hip/thigh/calf, RR hip/thigh/calf,
RL hip/thigh/calf. Neither interface has an acknowledgement or automatic ramp.
"""

import argparse
import math
import socket
import struct
import sys
import time


def send_gazebo(values: list[float], topic: str) -> int:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray

    rclpy.init()
    node = Node("gazebo_gain_alpha_sender")
    publisher = node.create_publisher(Float32MultiArray, topic, 10)
    try:
        deadline = time.monotonic() + 10.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            print(f"No rl_sim subscriber on {topic}; start Gazebo and rl_sim first.", file=sys.stderr)
            return 1
        message = Float32MultiArray(data=values)
        for _ in range(5):
            publisher.publish(message)
            time.sleep(0.05)
        print("Sent Gazebo gain alpha on", topic + ":", " ".join(f"{value:.3f}" for value in values))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send 12 Go2 gain scales in hardware order to Gazebo or onboard rl_real_go2."
    )
    parser.add_argument(
        "values", nargs=12, type=float,
        metavar="SCALE",
    )
    parser.add_argument("--port", type=int, default=5560)
    parser.add_argument("--gazebo", action="store_true", help="publish to rl_sim over ROS2 instead of real-robot UDP")
    parser.add_argument("--topic", default="/rl_sar/gain_alpha", help="Gazebo ROS2 gain topic")
    args = parser.parse_args()

    if not 1 <= args.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    if any(not math.isfinite(value) or value < 0.0 or value > 1.0 for value in args.values):
        parser.error("every scale must be finite and in [0.0, 1.0]")

    if args.gazebo:
        return send_gazebo(args.values, args.topic)

    packet = b"QGNA" + bytes((1,)) + struct.pack("!12f", *args.values)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(packet, ("127.0.0.1", args.port))
    print("Sent gain alpha (UDP, unacknowledged):", " ".join(f"{value:.3f}" for value in args.values))
    print("Confirm 'Applied local gain alpha' in rl_real_go2 output; its bridge must be enabled.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
