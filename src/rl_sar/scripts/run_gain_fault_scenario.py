#!/usr/bin/python3

import argparse
import random
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


JOINT_ORDER = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
]


def clamp01(value):
    return max(0.0, min(1.0, value))


class GainFaultScenarioPublisher(Node):
    def __init__(self, args):
        super().__init__("run_gain_fault_scenario")
        self.args = args
        self.publisher = self.create_publisher(Float32MultiArray, args.topic, 10)

    def run(self):
        rng = random.Random(self.args.seed)
        alpha_values = [1.0] * len(JOINT_ORDER)

        if self.args.clear:
            selected_indices = []
        else:
            selected_indices = self._select_joint_indices(rng)
            alpha = clamp01(self.args.alpha)
            for joint_id in selected_indices:
                alpha_values[joint_id] = alpha

        delay = self._sample_delay(rng)
        if delay > 0.0:
            self.get_logger().info(f"Robot will run normally for {delay:.2f}s before applying the gain fault.")
            time.sleep(delay)

        if self.args.clear:
            self.get_logger().info("Publishing healthy gain alpha values for all joints.")
        else:
            self.get_logger().warn("Applying RL gain fault:")
            for joint_id in selected_indices:
                self.get_logger().warn(
                    f"  index={joint_id} joint={JOINT_ORDER[joint_id]} "
                    f"rl_kp={alpha_values[joint_id]:.4f}*rl_kp_nominal "
                    f"rl_kd={alpha_values[joint_id]:.4f}*rl_kd_nominal"
                )

        msg = Float32MultiArray()
        msg.data = alpha_values

        period = 1.0 / self.args.publish_rate
        start_time = time.monotonic()
        while rclpy.ok():
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.args.hold > 0.0 and time.monotonic() - start_time >= self.args.hold:
                break
            time.sleep(period)

        self.get_logger().info("Gain fault scenario publisher finished.")

    def _sample_delay(self, rng):
        if self.args.delay is not None:
            return max(0.0, self.args.delay)
        delay_min = max(0.0, self.args.delay_min)
        delay_max = max(delay_min, self.args.delay_max)
        return rng.uniform(delay_min, delay_max)

    def _select_joint_indices(self, rng):
        selected = []

        for joint_id in self.args.joint_index:
            if joint_id < 0 or joint_id >= len(JOINT_ORDER):
                raise ValueError(
                    f"Invalid --joint-index {joint_id}. Valid range is 0 to {len(JOINT_ORDER) - 1}. "
                    "Example: the third entry is --joint-index 2."
                )
            selected.append(joint_id)

        for joint_name in self.args.joints:
            if joint_name not in JOINT_ORDER:
                raise ValueError(
                    "Unknown joint name: "
                    + joint_name
                    + "\nValid joints: "
                    + ", ".join(JOINT_ORDER)
                )
            selected.append(JOINT_ORDER.index(joint_name))

        if selected:
            return sorted(set(selected))

        count = max(1, min(self.args.random_count, len(JOINT_ORDER)))
        return sorted(rng.sample(range(len(JOINT_ORDER)), count))


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Apply Go2 RL gain faults during deployment by publishing per-joint alpha values. "
            "The deployment controller applies rl_kp_eff = alpha*rl_kp and rl_kd_eff = alpha*rl_kd."
        )
    )
    parser.add_argument(
        "joints",
        nargs="*",
        help="Joint names to damage. If omitted, random joints are selected.",
    )
    parser.add_argument(
        "--joint-index",
        type=int,
        action="append",
        default=[],
        help="0-based joint index to damage. Example: the third entry is --joint-index 2.",
    )
    parser.add_argument("--alpha", type=float, default=0.0, help="Gain multiplier for damaged joints.")
    parser.add_argument("--delay", type=float, default=None, help="Fixed seconds to wait before applying the fault.")
    parser.add_argument("--delay-min", type=float, default=10.0, help="Minimum random fault start time.")
    parser.add_argument("--delay-max", type=float, default=15.0, help="Maximum random fault start time.")
    parser.add_argument("--random-count", type=int, default=1, help="Number of random joints to damage if no joints are given.")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for repeatable delay/joint selection.")
    parser.add_argument("--topic", default="/rl_sar/gain_alpha", help="Gain alpha topic listened to by rl_sim.")
    parser.add_argument("--publish-rate", type=float, default=10.0, help="Publish rate while holding the fault.")
    parser.add_argument("--hold", type=float, default=0.0, help="Seconds to publish. 0 means publish until Ctrl+C.")
    parser.add_argument("--clear", action="store_true", help="Reset all joints to alpha=1.0.")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.publish_rate <= 0.0:
        print("--publish-rate must be positive", file=sys.stderr)
        return 2

    rclpy.init()
    node = GainFaultScenarioPublisher(args)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user.")
        return_code = 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return return_code


if __name__ == "__main__":
    sys.exit(main())
