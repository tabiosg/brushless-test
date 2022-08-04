import math
import asyncio
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
import lcm
import moteus
from rover_msgs import RAOpenLoopCmd, RAPosition

c0 = moteus.Controller(id=1)
c2 = moteus.Controller(id=2)
filtered_vel_0 = 0
filtered_vel_1 = 0


def turn_callback(channel, msg):
    struct_cmd = RAOpenLoopCmd.decode(msg)
    throttle_cmd_0 = struct_cmd.throttle[0]
    throttle_cmd_1 = struct_cmd.throttle[1]

    throttle_cmd_0 = max(min(throttle_cmd_0, 1), -1)
    throttle_cmd_1 = max(min(throttle_cmd_1, 1), -1)

    global filtered_vel_0, filtered_vel_1
    filtered_vel_0 = 3*throttle_cmd_0
    filtered_vel_1 = 3*throttle_cmd_1


async def loop():

    lcm_ = lcm.LCM()
    lcm_.subscribe("/ra_open_loop_cmd", turn_callback)

    global c0
    while True:
        lcm_.handle()
        state_0 = await c0.set_position(
            position=math.nan, velocity=filtered_vel_0, maximum_torque=0.5, watchdog_timeout=10, query=True
        )
        mode_0 = state_0.values[moteus.Register.MODE]
        if mode_0 == 1 or mode_0 == 11:
            await c0.set_stop()

        state_1 = await c1.set_position(
            position=math.nan, velocity=filtered_vel_1, maximum_torque=0.5, watchdog_timeout=10, query=True
        )
        mode_1 = state_1.values[moteus.Register.MODE]
        if mode_1 == 1 or mode_1 == 11:
            await c1.set_stop()

        print("Publishing RAPosition to /ra_position.")
        msg = RAPosition()
        msg.joint_a = state_0.values[moteus.Register.POSITION]
        msg.joint_b = state_1.values[moteus.Register.POSITION]
        msg.joint_c = state_0.values[moteus.Register.VELOCITY]
        msg.joint_d = state_1.values[moteus.Register.VELOCITY]
        msg.joint_e = state_0.values[moteus.Register.MODE]
        msg.joint_f = state_1.values[moteus.Register.MODE]
        lcm_.publish("/ra_position", msg.encode())


def main():
    asyncio.run(loop())


if __name__ == "__main__":
    main()