#!/usr/bin/python3

import argparse
import math
import sys

import rclpy
from gazebo_msgs.msg import ContactsState
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


DEFAULT_FOOT_TOPICS = [
    "/FL_foot_contact",
    "/FR_foot_contact",
    "/RL_foot_contact",
    "/RR_foot_contact",
]


def str_to_bool(value):
    if isinstance(value, bool):
        return value
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"expected a boolean value, got '{value}'")


class FootContactBridge(Node):
    def __init__(self, args):
        super().__init__("run_foot_contact_bridge")
        self.args = args
        self.contacts = [0.0] * 4
        self.force_norms = [0.0] * 4
        self.last_msg_time = [None] * 4
        self.ready = False
        self.waiting_logged = False

        self.publisher = self.create_publisher(Float32MultiArray, args.output_topic, 10)
        self.subscribers = []
        for foot_id, topic in enumerate(args.contact_topics):
            self.subscribers.append(
                self.create_subscription(
                    ContactsState,
                    topic,
                    lambda msg, foot_id=foot_id: self._contact_callback(foot_id, msg),
                    10,
                )
            )

        self.timer = self.create_timer(1.0 / args.publish_rate, self._publish_contacts)
        self.debug_timer = self.create_timer(1.0, self._print_debug)

        topic_text = ", ".join(args.contact_topics)
        self.get_logger().info(
            f"Foot contact bridge publishing {args.output_topic} in [FL, FR, RL, RR] order. "
            f"Contact threshold: {args.threshold:.3f} N. Source topics: {topic_text}"
        )

    def _contact_callback(self, foot_id, msg):
        force_norm = self._net_force_norm(msg)
        self.force_norms[foot_id] = force_norm
        self.contacts[foot_id] = 1.0 if force_norm > self.args.threshold else 0.0
        self.last_msg_time[foot_id] = self.get_clock().now()

    @staticmethod
    def _net_force_norm(msg):
        total_fx = 0.0
        total_fy = 0.0
        total_fz = 0.0

        for state in msg.states:
            force = state.total_wrench.force
            if FootContactBridge._force_is_nonzero(force):
                total_fx += force.x
                total_fy += force.y
                total_fz += force.z
            else:
                for wrench in state.wrenches:
                    total_fx += wrench.force.x
                    total_fy += wrench.force.y
                    total_fz += wrench.force.z

        return math.sqrt(total_fx * total_fx + total_fy * total_fy + total_fz * total_fz)

    @staticmethod
    def _force_is_nonzero(force):
        return abs(force.x) > 1.0e-6 or abs(force.y) > 1.0e-6 or abs(force.z) > 1.0e-6

    def _publish_contacts(self):
        if not self.ready:
            self.ready = all(last_time is not None for last_time in self.last_msg_time)
            if not self.ready:
                if not self.waiting_logged:
                    self.get_logger().warn(
                        "Waiting for all four Gazebo foot contact sensor topics before publishing /rl_sar/foot_contacts."
                    )
                    self.waiting_logged = True
                return
            self.get_logger().info("All four foot contact sensor topics are active; publishing force-based contacts.")

        now = self.get_clock().now()
        for foot_id, last_time in enumerate(self.last_msg_time):
            if (now - last_time).nanoseconds * 1.0e-9 > self.args.stale_timeout:
                self.contacts[foot_id] = 0.0
                self.force_norms[foot_id] = 0.0

        msg = Float32MultiArray()
        msg.data = list(self.contacts)
        self.publisher.publish(msg)

    def _print_debug(self):
        if not self.args.debug:
            return

        force_text = " ".join(f"{value:7.2f}" for value in self.force_norms)
        contact_text = " ".join(str(int(value)) for value in self.contacts)
        self.get_logger().info(
            f"force_norm_N [FL FR RL RR] = [{force_text}]  "
            f"contacts = [{contact_text}]"
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Bridge exact Gazebo foot contact sensor forces into the FTNet foot_contacts observation. "
            "Publishes Float32MultiArray in IsaacLab order [FL, FR, RL, RR]."
        )
    )
    parser.add_argument(
        "--contact-topics",
        nargs=4,
        default=DEFAULT_FOOT_TOPICS,
        metavar=("FL_TOPIC", "FR_TOPIC", "RL_TOPIC", "RR_TOPIC"),
        help="Gazebo ContactsState topics in IsaacLab foot order.",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=1.0,
        help="Contact force threshold in Newtons. A foot is contact=1 when force norm is greater than this value.",
    )
    parser.add_argument(
        "--output-topic",
        default="/rl_sar/foot_contacts",
        help="Foot contact observation topic listened to by rl_sim.",
    )
    parser.add_argument("--publish-rate", type=float, default=100.0, help="Output publish rate in Hz.")
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=0.2,
        help="Seconds without a contact message before a foot is treated as not in contact.",
    )
    parser.add_argument(
        "--debug",
        nargs="?",
        const=True,
        default=False,
        type=str_to_bool,
        help="Print force/contact values once per second. Accepts true/false when launched from ROS.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.publish_rate <= 0.0:
        print("--publish-rate must be positive", file=sys.stderr)
        return 2
    if args.threshold < 0.0:
        print("--threshold must be non-negative", file=sys.stderr)
        return 2

    rclpy.init()
    node = FootContactBridge(args)
    try:
        rclpy.spin(node)
        return_code = 0
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user.")
        return_code = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return return_code


if __name__ == "__main__":
    sys.exit(main())
