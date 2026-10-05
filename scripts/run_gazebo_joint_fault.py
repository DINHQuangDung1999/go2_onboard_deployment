#!/usr/bin/env python3
"""Inject a Gazebo actuator effectiveness fault through the ROS2 controller."""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


JOINTS = (
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
)
TOPIC = "/robot_joint_controller/fault_alpha"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("joint", help="Joint name, 'clear', or 'list'")
    parser.add_argument("alpha", nargs="?", type=float, help="Remaining torque fraction, 0 to 1")
    parser.add_argument("delay", nargs="?", type=float, default=0.0, help="Delay before fault in seconds")
    parser.add_argument("ramp", nargs="?", type=float, default=0.0, help="Ramp duration in seconds")
    args = parser.parse_args()

    if args.joint == "list":
        print("\n".join(JOINTS))
        return 0
    if args.joint != "clear" and args.joint not in JOINTS:
        parser.error(f"unknown joint: {args.joint}")
    if args.joint != "clear" and args.alpha is None:
        parser.error("alpha is required for a joint fault")
    if args.joint == "clear" and args.alpha is not None:
        parser.error("clear takes no additional arguments")
    if args.alpha is not None and (not math.isfinite(args.alpha) or not 0 <= args.alpha <= 1):
        parser.error("alpha must be finite and between 0 and 1")
    if not math.isfinite(args.delay) or args.delay < 0:
        parser.error("delay must be nonnegative")
    if not math.isfinite(args.ramp) or args.ramp < 0:
        parser.error("ramp must be nonnegative")

    rclpy.init()
    node = Node("gazebo_joint_fault_sender")
    publisher = node.create_publisher(Float32MultiArray, TOPIC, 10)
    try:
        deadline = time.monotonic() + 10.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            print(f"No controller subscriber on {TOPIC}; start Gazebo and rl_sim first.", file=sys.stderr)
            return 1

        if args.delay:
            print(f"Fault starts in {args.delay:g} s", flush=True)
            time.sleep(args.delay)

        index = JOINTS.index(args.joint) if args.joint != "clear" else None
        target = 1.0 if index is None else args.alpha
        start = time.monotonic()
        while True:
            elapsed = time.monotonic() - start
            blend = min(1.0, elapsed / args.ramp) if args.ramp else 1.0
            values = [1.0] * len(JOINTS)
            if index is not None:
                values[index] = 1.0 + blend * (target - 1.0)
            publisher.publish(Float32MultiArray(data=values))
            if blend >= 1.0:
                break
            time.sleep(0.05)

        # Repeat the final value so a controller activated during the ramp sees it.
        for _ in range(5):
            publisher.publish(Float32MultiArray(data=values))
            time.sleep(0.05)
        print(f"{args.joint}: actuator effectiveness {target:.2f} on {TOPIC}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
