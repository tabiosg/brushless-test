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
        self.velocity = 0
        self.state = None

    async def __enter__(self):
        await self.c.set_stop()

    async def __exit__(self, exc_type, exc_value, traceback):
        await self.c.set_stop()

    async def _set_velocity(self, speed: double) -> None:
        self.velocity = speed

    def update(self) -> None:
        self.state = await self.c.set_position(
            position=math.nan,
            velocity=speed,
            maximum_torque=0.3,
            stop_position=pos,
            watchdog_timeout=10,
            query=True,
        )

    def set_velocity_control_mode(self) -> None:
        self.position = math.nan

    def _print_position(self) -> None:
        print("Position: ", self.state.values[moteus.Register.POSITION])

    def _print_velocity(self) -> None:
        print("Velocity: ", self.state.values[moteus.Register.VELOCITY])

    def test_velocity(self) -> None:
        self.set_velocity_control_mode()
        for j in range(5):
            self.set_velocity(0.2)
            for i in range(50):
                self.update()
                await asyncio.sleep(0.02)
            self.set_velocity(0.0)
            for i in range(50):
                self.update()
                await asyncio.sleep(0.02)
            self.set_velocity(-0.2)
            for i in range(50):
                self.update()
                await asyncio.sleep(0.02)
            self.set_velocity(0.0)
            for i in range(50):
                self.update()
                await asyncio.sleep(0.02)

    def test_velocity_and_print_position(self) -> None:
        self.set_velocity_control_mode()
        for j in range(5):
            self.set_velocity(0.2)
            for i in range(50):
                self._print_position()
                self.update()
                await asyncio.sleep(0.02)
            self.set_velocity(0.0)
            for i in range(50):
                self._print_position()
                self.update()
                await asyncio.sleep(0.02)
            self.set_velocity(-0.2)
            for i in range(50):
                self._print_position()
                self.update()
                await asyncio.sleep(0.02)
            self.set_velocity(0.0)
            for i in range(50):
                self._print_position()
                self.update()
                await asyncio.sleep(0.02)

    def test_velocity_and_print_velocity(self) -> None:
        self.set_velocity_control_mode()
        for j in range(5):
            self.set_velocity(0.2)
            for i in range(50):
                self._print_velocity()
                self.update()
                await asyncio.sleep(0.02)
            self.set_velocity(0.0)
            for i in range(50):
                self._print_velocity()
                self.update()
                await asyncio.sleep(0.02)
            self.set_velocity(-0.2)
            for i in range(50):
                self._print_velocity()
                self.update()
                await asyncio.sleep(0.02)
            self.set_velocity(0.0)
            for i in range(50):
                self._print_velocity()
                self.update()
                await asyncio.sleep(0.02)


async def main():
    with MoteusBridge() as bridge:
        bridge.test_velocity()
        bridge.test_velocity_and_print_position()
        bridge.test_velocity_and_print_velocity()


if __name__ == "__main__":
    asyncio.run(main())
