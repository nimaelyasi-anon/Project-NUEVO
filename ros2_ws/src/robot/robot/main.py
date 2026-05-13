"""
manipulation.py — blocking pick sequence example
================================================
Demonstrates one simple manipulator sequence using:

1. Servo 1 as a gripper
2. Stepper 1 as a horizontal arm
3. DC Motor 3 in POSITION mode as a vertical lift

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/manipulation.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
Press BTN_1 to run one pick sequence:

  1. Raise the lift
  2. Open the gripper
  3. Lower the lift
  4. Close the gripper
  5. Raise the lift

This example uses a blocking sequence on purpose, so the code is easy to read.
During the sequence, the FSM stays inside one state until the whole sequence
finishes.

WHAT THIS TEACHES
-----------------
1. `step_home()` for stepper homing at startup
2. `set_servo()` for gripper angle control
3. `enable_motor(..., DCMotorMode.POSITION)` for M3 position control
4. `set_motor_position(..., max_vel_ticks=...)` to apply a velocity limit
5. Writing a simple blocking actuator sequence with `time.sleep(...)`
"""
from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    POSITION_UNIT,
    ServoChannel,
    StepMoveType,
    Stepper,
)
from robot.robot import FirmwareState, Robot


GRIPPER_SERVO = ServoChannel.CH_1
GRIPPER_OPEN_DEG = 0.0
GRIPPER_CLOSE_DEG = 40.0
GRIPPER_SETTLE_S = 1.0

LIFT_STEPPER = Stepper.STEPPER_1
LIFT_EXTEND_STEPS = -15000
LIFT_LOWER_STEPS = 3000
LIFT_BUFFER_STEPS = -1000
LIFT_MAX_VELOCITY = 5000
LIFT_ACCELERATION = 1200
LIFT_HOME_VELOCITY = 1000
LIFT_MOVE_TIMEOUT_S = 10.0

# Safety limit switch IDs — update these to match hardware ports
GRIPPER_LIMIT_ID = 1
LIFT_BOTTOM_LIMIT_ID = 2

# In this file, LIFT_LOWER_STEPS = +3000, so positive relative motion means DOWN.
LIFT_DOWN_IS_POSITIVE = True


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def home_lift(robot: Robot) -> bool:
    print("[FSM] HOMING — press BTN_3 to trigger the shared LIM1 input for stepper 1")
    robot.step_enable(LIFT_STEPPER)
    ok = robot.step_home(
        LIFT_STEPPER,
        direction=1,
        home_velocity=LIFT_HOME_VELOCITY,
        backoff_steps=50,
        blocking=True,
        timeout=15.0,
    )
    if not ok:
        print("[warn] arm homing timed out — check LIM1 or use BTN_3 to simulate it")
        robot.step_disable(LIFT_STEPPER)
        return False
    robot.step_disable(LIFT_STEPPER)
    return True


def safe_lift_move(robot: Robot, steps: int, move_type: StepMoveType) -> bool:
    is_downward = (
        move_type == StepMoveType.RELATIVE
        and (
            (LIFT_DOWN_IS_POSITIVE and steps > 0)
            or ((not LIFT_DOWN_IS_POSITIVE) and steps < 0)
        )
    )

    if is_downward and robot.get_limit(LIFT_BOTTOM_LIMIT_ID):
        print("[SAFETY] Lift bottom limit pressed — not moving downward.")
        return False

    return robot.step_move(
        LIFT_STEPPER,
        steps=steps,
        move_type=move_type,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    )


def safe_close_gripper(robot: Robot) -> bool:
    if robot.get_limit(GRIPPER_LIMIT_ID):
        print("[SAFETY] Gripper limit pressed — not closing further.")
        return False

    robot.set_servo(GRIPPER_SERVO, GRIPPER_CLOSE_DEG)
    time.sleep(GRIPPER_SETTLE_S)
    return True


def run_pick_sequence(robot: Robot) -> bool:
    robot.step_set_config(
        LIFT_STEPPER,
        max_velocity=LIFT_MAX_VELOCITY,
        acceleration=LIFT_ACCELERATION,
    )
    robot.enable_servo(GRIPPER_SERVO)
    robot.step_enable(LIFT_STEPPER)

    print("[SEQ] Raise lift")
    if not robot.step_move(
        LIFT_STEPPER,
        steps=LIFT_EXTEND_STEPS,
        move_type=StepMoveType.ABSOLUTE,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    ):
        print("[warn] arm failed to extend — check stepper enable or home limit wiring")
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False

    print("[SEQ] open gripper")
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    print("[SEQ] Lower lift")
    if not safe_lift_move(
        robot,
        steps=LIFT_LOWER_STEPS,
        move_type=StepMoveType.RELATIVE,
    ):
        print("[warn] lift lower blocked or failed")
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False

    print("[SEQ] close gripper")
    if not safe_close_gripper(robot):
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False

    print("[SEQ] Raise lift")
    if not robot.step_move(
        LIFT_STEPPER,
        steps=LIFT_BUFFER_STEPS,
        move_type=StepMoveType.RELATIVE,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    ):
        print("[warn] arm failed to raise — check stepper enable or home limit wiring")
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False

    robot.step_disable(LIFT_STEPPER)
    robot.disable_servo(GRIPPER_SERVO)
    return True


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            home_lift(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to run the pick sequence")
            print(f"[CFG] gripper open={GRIPPER_OPEN_DEG:.0f}° close={GRIPPER_CLOSE_DEG:.0f}°")
            print(f"[CFG] lift raised={LIFT_EXTEND_STEPS} steps home_vel={LIFT_HOME_VELOCITY} steps/s")
            print(f"[CFG] safety gripper_limit={GRIPPER_LIMIT_ID} lift_bottom_limit={LIFT_BOTTOM_LIMIT_ID}")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                print("[FSM] RUN_SEQUENCE")
                state = "RUN_SEQUENCE"

        elif state == "RUN_SEQUENCE":
            ok = run_pick_sequence(robot)
            show_idle_leds(robot)
            if ok:
                print("[FSM] IDLE — sequence complete")
            else:
                print("[FSM] IDLE — sequence stopped due to safety guard or actuator timeout")
            state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
