#!/usr/bin/env python3

import asyncio
import math
import moteus
import moteus_pi3hat


class MoteusBridge:
    """This will control the behavior of the Moteus"""

    def __init__(self) -> None:
        """
        Initialize the components.
        Start with a Default State
        """
        # If there were multiple motors connected, then
        # f would need to be modified
        f = moteus_pi3hat.Pi3HatRouter(
            servo_bus_map={1: [1]},
        )

        # This holds the controller object
        self.c = moteus.Controller(transport=f, id=1)
        self.position = math.nan
        self.stop_position = math.nan
        self.velocity = 0
        self.state = None

    async def __enter__(self):
        await self.c.set_stop()

    async def __exit__(self, exc_type, exc_value, traceback):
        await self.c.set_stop()

    def _set_velocity(self, input_vel: double) -> None:
        self.velocity = input_vel

    def update(self) -> None:
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
        self.stop_position=math.nan

    def set_position_control_mode(self) -> None:
        self.position = math.nan
        self.velocity = 0.2
        self.stop_position=math.nan

    def _set_position(self, input_pos) -> None:
        self.stop_position = input_pos

    def _print_position(self) -> None:
        print("Position: ", self.state.values[moteus.Register.POSITION])

    def _print_velocity(self) -> None:
        print("Velocity: ", self.state.values[moteus.Register.VELOCITY])

    def test_velocity(self) -> None:
        self.set_velocity_control_mode()
        velocities = [0.2, 0.0, -0.2, 0.0]
        for i in range(5):
            for velocity in velocities:
                self.set_velocity(velocity)
                for j in range(50):
                    self.update()
                    await asyncio.sleep(0.02)

    def test_velocity_and_print_position(self) -> None:
        self.set_velocity_control_mode()
        velocities = [0.2, 0.0, -0.2, 0.0]
        for i in range(5):
            for velocity in velocities:
                self.set_velocity(velocity)
                for j in range(50):
                    self._print_position()
                    self.update()
                    await asyncio.sleep(0.02)

    def test_velocity_and_print_velocity(self) -> None:
        self.set_velocity_control_mode()
        velocities = [0.2, 0.0, -0.2, 0.0]
        for i in range(5):
            for velocity in velocities:
                self.set_velocity(velocity)
                for j in range(50):
                    self._print_velocity()
                    self.update()
                    await asyncio.sleep(0.02)

    def test_position(self) -> None:
        self.set_position_control_mode()
        positions = [0.5, 0.0, -0.5, 0.0]
        for i in range(5):
            for position in positions:
                self._set_position(position)
                for j in range(50):
                    self.update()
                    await asyncio.sleep(0.02)

    def test_position_and_print_position(self) -> None:
        self.set_position_control_mode()
        positions = [0.5, 0.0, -0.5, 0.0]
        for i in range(5):
            for position in positions:
                self._set_position(position)
                for j in range(50):
                    self._print_position()
                    self.update()
                    await asyncio.sleep(0.02)

    def test_position_and_print_velocity(self) -> None:
        self.set_position_control_mode()
        positions = [0.5, 0.0, -0.5, 0.0]
        for i in range(5):
            for position in positions:
                self._set_position(position)
                for j in range(50):
                    self._print_velocity()
                    self.update()
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
