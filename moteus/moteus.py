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

    def set_position(self, pos, vel=0.0, max_tor=0.5, ff_tor=0.0, kp=1.0, kd=1.0) -> None:
        self.position = pos
        self.velocity = vel
        self.max_torque = max_tor
        self.ff_torque = ff_tor
        self.kp_scale = kp
        self.kd_scale = kd

    def set_velocity(self, vel=0.0, max_tor=0.5, ff_tor=0.0, kd=1.0) -> None:
        self.position = math.nan
        self.velocity = vel
        self.max_torque = max_tor
        self.ff_torque = ff_tor
        self.kp_scale = 0.0
        self.kd_scale = kd

    def set_torque(self, tor=0.0) -> None:
        self.position = math.nan
        self.velocity = 0
        self.max_torque = abs(tor)
        self.ff_torque = tor
        self.kp_scale = 0.0
        self.kd_scale = 0.0


class Controller:

    def __init__(self, servo_id, inversion) -> None:
        # NOTE: online it says that it connects to an arbitrary can port, prioritizing 
        # the fdcanbus. So I'd assume if we just hooked up can, it would just connect to that.
        # We can explore this later.
        self.id = servo_id
        self.c = moteus.Controller(id=servo_id)
        self.inversion = inversion
        self.state = None
        self.control_data = ControlData()

    def __enter__(self):
        self.c.make_stop()

    def __exit__(self, exc_type, exc_value, traceback):
        self.c.make_stop()

    def update(self) -> None:
        # TODO - add watchdog
        # TODO - add a don't do anything state (after stop is called)

        self.state = self.c.make_position(
            position=self.control_data.position,
            velocity=self.control_data.velocity * inversion,
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

        list_of_controllers = rospy.get_param("/motors/controllers")

        for controller in list_of_controllers:
            controller_name = controller[name]
            controller_id = controller[id]
            controller_inversion = controller_inversion
            id_by_controller_name[controller_name] = controller_id
            controller_object_by_controller_name[controller_id] = (
                Controller(controller_id, controller_inversion)
            )

    def is_controller_live(self, name) -> bool:
        return name == live_controller_name_by_id[id_by_controller_name[name]]

    def make_live(self, name) -> None:
        live_controller_name_by_id[id_by_controller_name[name]] = name


class ROSHandler:

    def __init__(self) -> None:
        self.controller_map = ControllerMap()
        self.moteus_bridge = MoteusBridge()

    def update_all_controllers(self) -> None:
        for can_id in self.controller_map.live_controller_name_by_id:
            controller_name = self.controller_map.live_controller_name_by_id[can_id]
            self.controller_map.controller_object_by_controller_name[controller_name].update()

    def set_controller_torque(name, torque) -> None:
        if not self.controller_map.is_controller_live(name):
            self.controller_map.make_live(name)
        self.controller_map.controller_object_by_controller_name.control_data.set_torque(tor=torque)

    def set_controller_velocity(name, velocity) -> None:
        if not self.controller_map.is_controller_live(name):
            self.controller_map.make_live(name)
        self.controller_map.controller_object_by_controller_name.control_data.set_velocity(vel=velocity)

    def set_controller_position(name, position) -> None:
        if not self.controller_map.is_controller_live(name):
            self.controller_map.make_live(name)
        self.controller_map.controller_object_by_controller_name.control_data.set_position(pos=position)

def main():
    ros_handler = ROSHandler()


if __name__ == "__main__":
    main()
