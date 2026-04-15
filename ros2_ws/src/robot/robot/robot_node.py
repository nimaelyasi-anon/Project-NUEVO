from __future__ import annotations

import signal
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32

from robot.robot import Robot


class RobotNode(Node):
    def __init__(self) -> None:
        super().__init__("robot")
        self.robot = Robot(self)

        self.ultrasonic_sub = self.create_subscription(
            Float32,
            "/ultrasonic_distance",
            self.on_ultrasonic_distance,
            10,
        )

        self.busy = False

        self.get_logger().info("robot node ready")

    def on_ultrasonic_distance(self, msg: Float32) -> None:
        if self.busy:
            return

        distance = msg.data
        level = self.distance_to_level(distance)

        if level is None:
            self.get_logger().info(
                f"Ultrasonic distance {distance:.1f} cm -> no valid level"
            )
            return

        self.get_logger().info(
            f"Ultrasonic distance {distance:.1f} cm -> target level {level}"
        )

        self.busy = True
        try:
            self.run_spatula_sequence(level)
        finally:
            self.busy = False

    def distance_to_level(self, distance_cm: float) -> int | None:
        if 5.0 <= distance_cm < 10.0:
            return 1
        if 10.0 <= distance_cm < 15.0:
            return 2
        if 15.0 <= distance_cm < 20.0:
            return 3
        return None

    def level_to_stepper_target(self, level: int) -> int:
        level_map = {
            1: 1000,
            2: 2000,
            3: 3000,
        }
        return level_map[level]

    def run_spatula_sequence(self, level: int) -> None:
        STEPPER_ID = 1
        SERVO_CHANNEL = 1

        REST_ANGLE = 90.0
        OPEN_ANGLE = 20.0
        CLOSED_ANGLE = 120.0

        target_steps = self.level_to_stepper_target(level)

        self.get_logger().info(f"Moving stepper {STEPPER_ID} to {target_steps} steps")
        self.robot.step_enable(STEPPER_ID)
        self.robot.step_move(STEPPER_ID, target_steps, move_type=0, blocking=True, timeout=5.0)

        self.get_logger().info(f"Setting servo {SERVO_CHANNEL} to OPEN angle {OPEN_ANGLE}")
        self.robot.enable_servo(SERVO_CHANNEL)
        self.robot.set_servo(SERVO_CHANNEL, OPEN_ANGLE)
        time.sleep(1.0)

        self.get_logger().info(f"Setting servo {SERVO_CHANNEL} to CLOSED angle {CLOSED_ANGLE}")
        self.robot.set_servo(SERVO_CHANNEL, CLOSED_ANGLE)
        time.sleep(1.0)

        self.get_logger().info(f"Returning servo {SERVO_CHANNEL} to REST angle {REST_ANGLE}")
        self.robot.set_servo(SERVO_CHANNEL, REST_ANGLE)
        time.sleep(1.0)


def _safe_log(node: Node, level: str, message: str) -> None:
    try:
        getattr(node.get_logger(), level)(message)
    except Exception:
        pass


def _raise_keyboard_interrupt(signum, frame) -> None:
    raise KeyboardInterrupt()


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RobotNode()

    def _spin() -> None:
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    old_sigint = signal.getsignal(signal.SIGINT)
    old_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    try:
        from robot.main import run
        run(node.robot)
    except KeyboardInterrupt:
        _safe_log(node, "info", "robot node interrupted; shutting down")
    finally:
        try:
            node.robot.shutdown()
        except Exception as exc:
            _safe_log(node, "error", f"robot shutdown failed: {exc}")
        try:
            signal.signal(signal.SIGINT, old_sigint)
            signal.signal(signal.SIGTERM, old_sigterm)
        except Exception:
            pass
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            spin_thread.join(timeout=2.0)
