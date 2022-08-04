import math
import asyncio
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
import lcm
import moteus
from rover_msgs import RAOpenLoopCmd

c = moteus.Controller()
filtered_vel = 0


def turn_callback(channel, msg):
    struct_cmd = RAOpenLoopCmd.decode(msg)
    throttle_cmd = struct_cmd.throttle[0]

    throttle_cmd = max(min(throttle_cmd, 1), -1)

    global filtered_vel
    filtered_vel = 3*throttle_cmd


async def loop():

    lcm_ = lcm.LCM()
    lcm_.subscribe("/ra_open_loop_cmd", turn_callback)
    # run_coroutines(lcm_.loop(), receive(lcm_))

    global c
    while True:
        lcm_.handle()
        state = await c.set_position(
            position=math.nan, velocity=filtered_vel, maximum_torque=0.5, watchdog_timeout=10, query=True
        )
        print(f"Position: {state.values[moteus.Register.POSITION]}")
        mode = state.values[moteus.Register.MODE]
        if mode == 1 or mode == 11:
            await c.set_stop()
            print(f"Mode: {mode}")


def main():
    asyncio.run(loop())


if __name__ == "__main__":
    main()