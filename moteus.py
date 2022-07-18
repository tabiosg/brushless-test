#!/usr/bin/env python3

import asyncio
import math

import moteus
import moteus_pi3hat


class Controller:

    def __init__(self, transport, servo_id) -> None:
        self.c = moteus.Controller(transport=transport, id=servo_id)
        self.position = math.nan
        self.stop_position = math.nan
        self.velocity = 0
        self.state = None

    async def __enter__(self):
        await self.c.set_stop()

    async def __exit__(self, exc_type, exc_value, traceback):
        await self.c.set_stop()

    def set_velocity(self, input_vel: float) -> None:
        self.velocity = input_vel

    async def update(self) -> None:
        self.state = await self.c.set_position(
            position=self.position,
            velocity=self.velocity,
            maximum_torque=0.3,
            stop_position=self.stop_position,
            watchdog_timeout=10,
            query=True,
        )

    def set_velocity_control_mode(self) -> None:
        self.position = math.nan
        self.velocity = 0
        self.stop_position = math.nan

    def set_position_control_mode(self) -> None:
        self.position = math.nan
        self.velocity = 0.2
        self.stop_position = math.nan

    def set_position(self, input_pos) -> None:
        self.stop_position = input_pos

    def print_position(self) -> None:
        print("Position: ", self.state.values[moteus.Register.POSITION])

    def print_velocity(self) -> None:
        print("Velocity: ", self.state.values[moteus.Register.VELOCITY])


class MoteusBridge:
    """This will control the behavior of the Moteus"""

    def __init__(self) -> None:
        """
        Initialize the components.
        Start with a Default State
        """
        ids = [1, 0, 0, 0, 0]  # 0 means nothing 

        # If there were multiple motors connected, then
        # f would need to be modified
        f = moteus_pi3hat.Pi3HatRouter(
                servo_bus_map={
                1:[ids[0]],
                2:[ids[1]],
                3:[ids[2]],
                4:[ids[3]],
                5:[ids[4]],
            },
        )

        # This holds the controller object
        self.servos = [
            Controller(f, ids[0]),
            Controller(f, ids[1]),
            Controller(f, ids[2]),
            Controller(f, ids[3]),
            Controller(f, ids[4]),
        ]

    async def test_velocity(self) -> None:
        for servo in servos:
            servo.set_velocity_control_mode()
        velocities = [0.2, 0.0, -0.2, 0.0]
        for i in range(5):
            for velocity in velocities:
                for servo in servos:
                    servo.set_velocity(velocity)
                for j in range(50):
                    for servo in servos:
                        servo.update()
                    await asyncio.sleep(0.02)

    async def test_velocity_and_print_position(self) -> None:
        for servo in servos:
            servo.set_velocity_control_mode()
        velocities = [0.2, 0.0, -0.2, 0.0]
        for i in range(5):
            for velocity in velocities:
                for servo in servos:
                    servo.set_velocity(velocity)
                for j in range(50):
                    for servo in servos:
                        servo.print_position()
                        servo.update()
                    await asyncio.sleep(0.02)

    async def test_velocity_and_print_velocity(self) -> None:
        for servo in servos:
            servo.set_velocity_control_mode()
        velocities = [0.2, 0.0, -0.2, 0.0]
        for i in range(5):
            for velocity in velocities:
                for servo in servos:
                    servo.set_velocity(velocity)
                for j in range(50):
                    for servo in servos:
                        servo.print_velocity()
                        servo.update()
                    await asyncio.sleep(0.02)

    async def test_position(self) -> None:
        for servo in servos:
            servo.set_position_control_mode()
        positions = [0.5, 0.0, -0.5, 0.0]
        for i in range(5):
            for position in positions:
                for servo in servos:
                    servo.set_position(position)
                for j in range(50):
                    for servo in servos:
                        servo.update()
                    await asyncio.sleep(0.02)

    async def test_position_and_print_position(self) -> None:
        for servo in servos:
            servo.set_position_control_mode()
        positions = [0.5, 0.0, -0.5, 0.0]
        for i in range(5):
            for position in positions:
                for servo in servos:
                    servo.set_position(position)
                for j in range(50):
                    for servo in servos:
                        servo.print_position()
                        servo.update()
                    await asyncio.sleep(0.02)

    async def test_position_and_print_velocity(self) -> None:
        for servo in servos:
            servo.set_position_control_mode()
        positions = [0.5, 0.0, -0.5, 0.0]
        for i in range(5):
            for position in positions:
                for servo in servos:
                    servo.set_position(position)
                for j in range(50):
                    for servo in servos:
                        servo.print_velocity()
                        servo.update()
                    await asyncio.sleep(0.02)


async def main():
    with MoteusBridge() as bridge:
        bridge.test_position()
        bridge.test_position_and_print_position()
        bridge.test_position_and_print_velocity()
        bridge.test_velocity()
        bridge.test_velocity_and_print_position()
        bridge.test_velocity_and_print_velocity()


if __name__ == "__main__":
    asyncio.run(main())
