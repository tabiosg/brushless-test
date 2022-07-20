#!/usr/bin/env python3

import time
import math

import moteus
import moteus_pi3hat

from typing import Dict

class ControlData:

    def __init__(self) -> None:
        self.position = math.nan
        self.velocity = 0.0
        self.max_torque = 0.5
        self.ff_torque = 0.0
        self.kp_scale = 0.0
        self.kd_scale = 1.0
        self.set_velocity(0.0)

    def set_position(self, input_pos) -> None:
        self.position = input_pos
        self.velocity = 0.0
        self.max_torque = 0.5
        self.ff_torque = 0.0
        self.kp_scale = 1.0
        self.kd_scale = 1.0

    def set_velocity(self, input_vel) -> None:
        self.position = math.nan
        self.velocity = input_vel
        self.max_torque = 0.5
        self.ff_torque = 0.0
        self.kp_scale = 0.0
        self.kd_scale = 1.0

    def set_torque(self, input_torque) -> None:
        self.position = math.nan
        self.velocity = 0
        self.max_torque = abs(input_torque)
        self.ff_torque = input_torque
        self.kp_scale = 0.0
        self.kd_scale = 0.0


class Controller:

    def __init__(self, servo_id) -> None:
        self.c = moteus.Controller(id=servo_id)
        self.state = None
        self.control_data = ControlData()

    def __enter__(self):
        self.c.make_stop()

    def __exit__(self, exc_type, exc_value, traceback):
        self.c.make_stop()

    def update(self) -> None:
        self.state = self.c.make_position(
            position=self.control_data.position,
            velocity=self.control_data.velocity,
            feedforward_torque=self.control_data.ff_torque,
            kp_scale=self.control_data.kp_scale,
            kd_scale=self.control_data.kd_scale,
            maximum_torque=self.control_data.max_torque,
            watchdog_timeout=10,
            query=True,
        )

    def print_position(self) -> None:
        print("Position: ", self.state.values[moteus.Register.POSITION])

    def print_velocity(self) -> None:
        print("Velocity: ", self.state.values[moteus.Register.VELOCITY])


class ControllerMap:
    live_controller_name_by_id: Dict[int, str]
    id_by_controller_name: Dict[str, int]
    controller_object_by_controller_name: Dict[str, Controller]

    def __init__(self) -> None:
        self.live_controller_name_by_id = {}
        self.id_by_controller_name = {}
        self.controller_object_by_controller_name = {}

    def check_if_live(self, name) -> bool:
        return name == live_controller_name_by_id[id_by_controller_name[name]]

    def make_live(self, name) -> None:
        live_controller_name_by_id[id_by_controller_name[name]] = name


class MoteusBridge:
    """This will control the behavior of the Moteus"""

    def __init__(self) -> None:
        """
        Initialize the components.
        Start with a Default State
        """
        ids = [1, 0, 0, 0, 0]  # 0 means nothing 

        # This holds the controller object
        self.servos = [
            Controller(ids[0]),
            Controller(ids[1]),
            Controller(ids[2]),
            Controller(ids[3]),
            Controller(ids[4]),
        ]

    def test_velocity(self) -> None:
        velocities = [0.2, 0.0, -0.2, 0.0]
        for i in range(5):
            for velocity in velocities:
                for servo in servos:
                    servo.set_velocity(velocity)
                for j in range(50):
                    for servo in servos:
                        servo.update()
                    sleep(0.02)

    def test_velocity_and_print_position(self) -> None:
        velocities = [0.2, 0.0, -0.2, 0.0]
        for i in range(5):
            for velocity in velocities:
                for servo in servos:
                    servo.set_velocity(velocity)
                for j in range(50):
                    for servo in servos:
                        servo.print_position()
                        servo.update()
                    time.sleep(0.02)

    def test_velocity_and_print_velocity(self) -> None:
        velocities = [0.2, 0.0, -0.2, 0.0]
        for i in range(5):
            for velocity in velocities:
                for servo in servos:
                    servo.set_velocity(velocity)
                for j in range(50):
                    for servo in servos:
                        servo.print_velocity()
                        servo.update()
                    time.sleep(0.02)

    def test_position(self) -> None:
        positions = [0.5, 0.0, -0.5, 0.0]
        for i in range(5):
            for position in positions:
                for servo in servos:
                    servo.set_position(position)
                for j in range(50):
                    for servo in servos:
                        servo.update()
                    time.sleep(0.02)

    def test_position_and_print_position(self) -> None:
        positions = [0.5, 0.0, -0.5, 0.0]
        for i in range(5):
            for position in positions:
                for servo in servos:
                    servo.set_position(position)
                for j in range(50):
                    for servo in servos:
                        servo.print_position()
                        servo.update()
                    time.sleep(0.02)

    def test_position_and_print_velocity(self) -> None:
        positions = [0.5, 0.0, -0.5, 0.0]
        for i in range(5):
            for position in positions:
                for servo in servos:
                    servo.set_position(position)
                for j in range(50):
                    for servo in servos:
                        servo.print_velocity()
                        servo.update()
                    time.sleep(0.02)


def main():
    with MoteusBridge() as bridge:
        bridge.test_position()
        bridge.test_position_and_print_position()
        bridge.test_position_and_print_velocity()
        bridge.test_velocity()
        bridge.test_velocity_and_print_position()
        bridge.test_velocity_and_print_velocity()


if __name__ == "__main__":
    main()
