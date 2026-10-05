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
LEG_ORDER = ["FL", "FR", "RL", "RR"]


def clamp01(value):
    return max(0.0, min(1.0, value))


class FaultScenarioPublisher(Node):
    def __init__(self, args):
        super().__init__("run_fault_scenario")
        self.args = args
        self.publisher = self.create_publisher(Float32MultiArray, args.topic, 10)
        self.tuck_publisher = self.create_publisher(Float32MultiArray, args.tuck_topic, 10)

    def run(self):
        rng = random.Random(self.args.seed)
        selected_joints = self._select_joints(rng)
        alpha_values = [1.0] * len(JOINT_ORDER)

        if self.args.clear:
            selected_joints = []
        else:
            for joint_name in selected_joints:
                joint_id = JOINT_ORDER.index(joint_name)
                alpha_values[joint_id] = self._sample_alpha(rng)

        if self.args.delay > 0.0:
            self.get_logger().info(f"Robot will run normally for {self.args.delay:.2f}s before applying the fault.")
            time.sleep(self.args.delay)

        tucked_leg_values = [1.0] * len(LEG_ORDER)
        if self.args.mode == "tucked-leg-lock" and not self.args.clear:
            selected_legs = {joint_name.split("_", maxsplit=1)[0] for joint_name in selected_joints}
            if len(selected_legs) != 1:
                raise ValueError("tucked-leg-lock requires joints from exactly one leg")
            selected_leg = selected_legs.pop()
            tucked_leg_values[LEG_ORDER.index(selected_leg)] = 0.0
            # The trained tuck model requires working position actuators. Clear
            # any previously latched actuator-loss test before requesting it.
            alpha_values = [1.0] * len(JOINT_ORDER)

        if self.args.clear:
            self.get_logger().info("Publishing healthy alpha values for all joints.")
        elif self.args.mode == "tucked-leg-lock":
            self.get_logger().warn("Applying IsaacLab-matched Gazebo tuck-and-lock fault:")
            self.get_logger().warn(f"  {selected_leg} leg: smooth tuck, then position hold")
        else:
            self.get_logger().warn("Applying Gazebo actuator fault:")
            for joint_name in selected_joints:
                joint_id = JOINT_ORDER.index(joint_name)
                self.get_logger().warn(f"  {joint_name}: alpha={alpha_values[joint_id]:.4f}")

        msg = Float32MultiArray()
        msg.data = alpha_values
        tuck_msg = Float32MultiArray()
        tuck_msg.data = tucked_leg_values

        period = 1.0 / self.args.publish_rate
        start_time = time.monotonic()
        while rclpy.ok():
            self.publisher.publish(msg)
            self.tuck_publisher.publish(tuck_msg)
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.args.hold > 0.0 and time.monotonic() - start_time >= self.args.hold:
                break
            time.sleep(period)

        self.get_logger().info("Fault scenario publisher finished.")

    def _select_joints(self, rng):
        requested = list(self.args.joints)
        if requested and requested[0] in ("joint_broken", "signal_lost"):
            requested = requested[1:]

        if requested:
            invalid = [joint_name for joint_name in requested if joint_name not in JOINT_ORDER]
            if invalid:
                raise ValueError(
                    "Unknown joint name(s): "
                    + ", ".join(invalid)
                    + "\nValid joints: "
                    + ", ".join(JOINT_ORDER)
                )
            return requested

        count = max(1, min(self.args.random_count, len(JOINT_ORDER)))
        return rng.sample(JOINT_ORDER, count)

    def _sample_alpha(self, rng):
        if self.args.alpha is not None:
            return clamp01(self.args.alpha)
        alpha_min = clamp01(self.args.alpha_min)
        alpha_max = max(alpha_min, clamp01(self.args.alpha_max))
        return rng.uniform(alpha_min, alpha_max)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Apply actuator-loss or IsaacLab-matched tuck-and-lock faults to Go2 in Gazebo."
    )
    parser.add_argument(
        "joints",
        nargs="*",
        help="Joint names to break. If omitted, random joints are selected. The old 'joint_broken' prefix is accepted.",
    )
    parser.add_argument("--delay", type=float, default=10.0, help="Seconds to wait before applying the fault.")
    parser.add_argument(
        "--mode",
        choices=("actuator-loss", "tucked-leg-lock"),
        default="actuator-loss",
        help="Fault mechanics. Use tucked-leg-lock to reproduce the Stage B5 IsaacLab test.",
    )
    parser.add_argument("--alpha", type=float, default=None, help="Fixed alpha for all selected joints.")
    parser.add_argument("--alpha-min", type=float, default=0.0, help="Minimum random alpha for selected joints.")
    parser.add_argument("--alpha-max", type=float, default=0.02, help="Maximum random alpha for selected joints.")
    parser.add_argument("--random-count", type=int, default=1, help="Number of random joints to fault if no joints are given.")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for repeatable joint/alpha selection.")
    parser.add_argument("--topic", default="/robot_joint_controller/fault_alpha", help="Fault alpha topic.")
    parser.add_argument("--tuck-topic", default="/rl_sar/tucked_leg_fault", help="Gazebo tuck-and-lock topic.")
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
    node = FaultScenarioPublisher(args)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user.")
        return_code = 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return_code = 1
    else:
        return_code = 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return return_code


if __name__ == "__main__":
    sys.exit(main())
